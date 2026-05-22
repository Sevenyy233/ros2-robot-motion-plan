import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from grid_map_msgs.msg import GridMap
from geometry_msgs.msg import PoseStamped
import tf2_ros
import numpy as np
import math

class NewLocalPlannerNode(Node):
    """
    基于空间轨迹采样 (Spatial Trajectory Rollout) 的局部路径规划器。
    它不计算速度，只输出一条几何上无碰撞、坡度安全的 nav_msgs/Path，供下游的轨迹规划器使用。
    """
    def __init__(self):
        super().__init__('new_local_planner')

        # 参数配置
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        # 采样参数
        self.declare_parameter('lookahead_dist', 4.0)       # 向前采样的距离 (米)
        self.declare_parameter('max_curvature', 1.5)        # 最大采样曲率 (rad/m)
        self.declare_parameter('num_paths', 21)             # 采样的圆弧数量
        self.declare_parameter('points_per_path', 20)       # 每条圆弧上的离散点数
        
        # 安全与代价参数
        self.declare_parameter('max_obstacle_height', 0.2)  # 能跨越的最大障碍物高度 (相对 base_link 的 Z)
        self.declare_parameter('max_slope_angle', 25.0)     # 最大允许坡度 (度)
        self.declare_parameter('goal_weight', 1.0)          # 距离全局目标点近的权重
        self.declare_parameter('curvature_weight', 0.5)     # 惩罚大曲率转向的权重

        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.max_curvature = self.get_parameter('max_curvature').value
        self.num_paths = self.get_parameter('num_paths').value
        self.points_per_path = self.get_parameter('points_per_path').value
        self.max_obstacle_height = self.get_parameter('max_obstacle_height').value
        self.max_slope_angle = self.get_parameter('max_slope_angle').value
        self.goal_weight = self.get_parameter('goal_weight').value
        self.curvature_weight = self.get_parameter('curvature_weight').value

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 状态数据
        self.global_path = None
        self.local_grid_data = None
        self.local_cost_data = None
        self.local_grid_info = None

        # 订阅与发布
        self.sub_global_path = self.create_subscription(Path, '/global_path', self.global_path_callback, 1)
        self.sub_local_map = self.create_subscription(GridMap, '/local_grid_map', self.local_map_callback, 1)
        self.pub_local_path = self.create_publisher(Path, '/local_path', 1)

        # 控制循环
        self.timer = self.create_timer(0.1, self.plan_loop) # 10Hz

        self.get_logger().info("基于空间采样的 3D 局部规划器初始化完成 (纯空间解耦版)")

    def global_path_callback(self, msg):
        self.global_path = msg

    def local_map_callback(self, msg):
        try:
            elev_idx = msg.layers.index("elevation")
            cost_idx = msg.layers.index("cost")
        except ValueError:
            return
            
        self.local_grid_info = msg.info
        elev_array = msg.data[elev_idx]
        cost_array = msg.data[cost_idx]
        
        # 按照 GridMap 列主序解析
        size_x = int(round(msg.info.length_x / msg.info.resolution))
        size_y = int(round(msg.info.length_y / msg.info.resolution))
        for dim in elev_array.layout.dim:
            if dim.label == "column_index": size_y = dim.size
            elif dim.label == "row_index": size_x = dim.size
            
        if len(elev_array.data) == size_x * size_y:
            self.local_grid_data = np.array(elev_array.data, dtype=np.float32).reshape((size_x, size_y), order="F")
            self.local_cost_data = np.array(cost_array.data, dtype=np.float32).reshape((size_x, size_y), order="F")

    def get_elevation_and_cost(self, x, y):
        """ 查询局部地图中某点的高程和代价 (x, y 为 base_link 坐标) """
        if self.local_grid_data is None or self.local_cost_data is None or self.local_grid_info is None:
            return 0.0, 150.0 # 没地图时假设平坦但有一定代价

        res = self.local_grid_info.resolution
        max_x = self.local_grid_info.length_x / 2.0
        max_y = self.local_grid_info.length_y / 2.0

        ix = int((max_x - x) / res)
        iy = int((max_y - y) / res)

        if 0 <= ix < self.local_grid_data.shape[0] and 0 <= iy < self.local_grid_data.shape[1]:
            z = self.local_grid_data[ix, iy]
            c = self.local_cost_data[ix, iy]
            return float(z), float(c)
            
        return 0.0, 150.0

    def generate_arcs(self):
        """ 在 base_link 坐标系下生成一组候选圆弧路径 """
        arcs = []
        curvatures = np.linspace(-self.max_curvature, self.max_curvature, self.num_paths)
        s_vals = np.linspace(0, self.lookahead_dist, self.points_per_path)

        for k in curvatures:
            arc_pts = []
            for s in s_vals:
                if abs(k) < 1e-4: # 直线
                    x = s
                    y = 0.0
                else: # 圆弧
                    r = 1.0 / k
                    x = r * math.sin(k * s)
                    y = r * (1.0 - math.cos(k * s))
                arc_pts.append((x, y))
            arcs.append({'curvature': k, 'points': arc_pts})
        return arcs

    def get_local_goal(self):
        """ 找到全局路径上距离当前机器人 lookahead_dist 左右的目标点，并转换到 base_link """
        if not self.global_path or not self.global_path.poses:
            return None

        try:
            t_map_base = self.tf_buffer.lookup_transform(
                self.base_frame, self.map_frame, rclpy.time.Time())
        except Exception:
            return None

        tx = t_map_base.transform.translation.x
        ty = t_map_base.transform.translation.y
        q = t_map_base.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy]
        ])

        # 寻找距离 base_link 原点距离最接近 lookahead_dist 的点
        best_pt_base = None
        min_diff = float('inf')

        for pose_stamped in self.global_path.poses:
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            
            # 转到 base_link
            pt_map = np.array([px, py, 0.0])
            pt_base = R @ pt_map + np.array([tx, ty, t_map_base.transform.translation.z])
            
            # 只看前方的点
            if pt_base[0] < 0:
                continue

            d = math.hypot(pt_base[0], pt_base[1])
            if abs(d - self.lookahead_dist) < min_diff:
                min_diff = abs(d - self.lookahead_dist)
                best_pt_base = (pt_base[0], pt_base[1])

        # 如果全局路径点都在极近的地方，取最后一个点
        if best_pt_base is None:
            last_p = self.global_path.poses[-1].pose.position
            pt_map = np.array([last_p.x, last_p.y, 0.0])
            pt_base = R @ pt_map + np.array([tx, ty, t_map_base.transform.translation.z])
            best_pt_base = (pt_base[0], pt_base[1])

        return best_pt_base

    def evaluate_arc(self, arc, local_goal):
        """ 评估一条圆弧的代价值，若发生碰撞或坡度过大则返回无穷大 """
        points = arc['points']
        
        # 1. 安全检查 (Cost 层致命惩罚)
        path_cost = 0.0
        for i in range(len(points)):
            x, y = points[i]
            z, cost = self.get_elevation_and_cost(x, y)
            
            # 如果碰到了致命代价 (障碍物或者陡坡)
            if cost >= 250.0:
                return float('inf')
                
            path_cost += cost
            
        # 2. 启发式代价计算 (距离目标点越近越好，曲率越小越好)
        end_x, end_y = points[-1]
        goal_x, goal_y = local_goal
        
        dist_to_goal = math.hypot(end_x - goal_x, end_y - goal_y)
        
        # 总代价 = 路径平均障碍代价 + 距离目标代价 + 曲率惩罚
        avg_path_cost = path_cost / len(points)
        
        total_cost = (avg_path_cost * 0.1) + \
                     (self.goal_weight * dist_to_goal) + \
                     (self.curvature_weight * abs(arc['curvature']))
        
        return total_cost

    def plan_loop(self):
        if not self.global_path or not self.local_grid_data is not None:
            return

        local_goal = self.get_local_goal()
        if not local_goal:
            return

        arcs = self.generate_arcs()
        
        best_arc = None
        best_cost = float('inf')

        for arc in arcs:
            cost = self.evaluate_arc(arc, local_goal)
            if cost < best_cost:
                best_cost = cost
                best_arc = arc

        if best_arc is None:
            self.get_logger().warn("找不到安全的局部路径！", throttle_duration_sec=2.0)
            return

        # 将最优圆弧转换回 map 坐标系发布
        self.publish_local_path(best_arc['points'])

    def publish_local_path(self, base_points):
        try:
            t_base_map = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return

        tx = t_base_map.transform.translation.x
        ty = t_base_map.transform.translation.y
        tz = t_base_map.transform.translation.z
        q = t_base_map.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        
        R = np.array([
            [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy]
        ])

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame

        for (x, y) in base_points:
            z, _ = self.get_elevation_and_cost(x, y)
            pt_base = np.array([x, y, z])
            pt_map = R @ pt_base + np.array([tx, ty, tz])

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = pt_map[0]
            pose.pose.position.y = pt_map[1]
            pose.pose.position.z = pt_map[2]
            # 简化：路径点的朝向暂时设为无旋转，轨迹规划器通常只关心位置
            pose.pose.orientation.w = 1.0 
            
            path_msg.poses.append(pose)

        self.pub_local_path.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NewLocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
