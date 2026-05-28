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
    基于启发式横向采样的局部路径规划器 (解耦版)。
    它沿用原 local_planner_node.py 的避障逻辑，但不计算速度，只输出 nav_msgs/Path。
    使用 /local_grid_map (Submap) 的 cost 层进行碰撞检测。
    """
    def __init__(self):
        super().__init__('new_local_planner')

        # 参数配置
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        self.declare_parameter('lookahead_min', 0.3)
        self.declare_parameter('lookahead_max', 2.0)
        self.declare_parameter('local_path_lookahead', 3.0)
        self.declare_parameter('obstacle_check_ahead', 3.0)
        self.declare_parameter('max_lateral_offset', 1.5)
        self.declare_parameter('avoidance_sample_step', 0.1)

        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        self.lookahead_min = self.get_parameter('lookahead_min').value
        self.lookahead_max = self.get_parameter('lookahead_max').value
        self.local_path_lookahead = self.get_parameter('local_path_lookahead').value
        self.obstacle_check_ahead = self.get_parameter('obstacle_check_ahead').value
        self.max_lateral_offset = self.get_parameter('max_lateral_offset').value
        self.avoidance_sample_step = self.get_parameter('avoidance_sample_step').value

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 状态数据
        self.global_path = None
        self.local_grid_data = None
        self.local_cost_data = None
        self.local_grid_info = None
        
        self.current_pose = None
        self.current_yaw = 0.0
        self.avoidance_path = None

        # 订阅与发布
        self.sub_global_path = self.create_subscription(Path, '/global_path', self.global_path_callback, 1)
        self.sub_local_map = self.create_subscription(GridMap, '/local_grid_map', self.local_map_callback, 1)
        self.pub_local_path = self.create_publisher(Path, '/local_path', 1)

        # 控制循环
        self.timer = self.create_timer(0.1, self.plan_loop) # 10Hz

        self.get_logger().info("启发式横向采样局部规划器初始化完成!")

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
            return 0.0, 150.0 

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

    def update_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
            self.current_pose = (
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
            )
            q = trans.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny, cosy)
            return True
        except Exception:
            return False

    def distance_xy(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def check_line_collision(self, x1, y1, x2, y2):
        """ 检查 base_link 下两点连线是否安全 (基于 cost 层) """
        if self.local_grid_info is None:
            return False # 没地图默认安全
            
        res = self.local_grid_info.resolution
        steps = max(1, int(math.hypot(x2 - x1, y2 - y1) / (res * 0.5)))
        for s in range(steps + 1):
            t = s / max(steps, 1)
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            _, cost = self.get_elevation_and_cost(x, y)
            if cost >= 250.0:
                return True # 发生碰撞
        return False

    def find_avoidance_lookahead(self, orig_lookahead):
        """ 沿用原算法的启发式横向采样避障逻辑 """
        cx, cy, cz = self.current_pose
        lx, ly, lz = orig_lookahead
        
        # 将原前视点转换到 base_link
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        dx_map = lx - cx
        dy_map = ly - cy
        
        ld = math.hypot(dx_map, dy_map)
        if ld < 1e-6:
            return orig_lookahead

        # 原前视点在 base_link 下的坐标
        base_lx = cos_yaw * dx_map + sin_yaw * dy_map
        base_ly = -sin_yaw * dx_map + cos_yaw * dy_map

        fwd_x = base_lx / ld
        fwd_y = base_ly / ld
        perp_x = -fwd_y
        perp_y = fwd_x

        # 检查原始直线是否碰撞
        if not self.check_line_collision(0.0, 0.0, base_lx, base_ly):
            self.avoidance_path = None
            return orig_lookahead

        best_base_pt = None
        best_score = float("inf")
        check_dist = min(ld, self.obstacle_check_ahead)

        for sign in (-1.0, 1.0):
            offset = sign * self.avoidance_sample_step
            while abs(offset) <= self.max_lateral_offset:
                ax = fwd_x * check_dist + perp_x * offset
                ay = fwd_y * check_dist + perp_y * offset

                if not self.check_line_collision(0.0, 0.0, ax, ay):
                    score = abs(offset) + math.hypot(ax - base_lx, ay - base_ly)
                    if score < best_score:
                        best_score = score
                        best_base_pt = (ax, ay)
                    break
                offset += sign * self.avoidance_sample_step

        if best_base_pt is None:
            self.get_logger().warn("避障采样未找到可行路径，尝试小步避障", throttle_duration_sec=1.0)
            for sign in (-1.0, 1.0):
                offset = sign * self.avoidance_sample_step
                while abs(offset) <= self.max_lateral_offset * 0.5:
                    ax = fwd_x * self.lookahead_min + perp_x * offset
                    ay = fwd_y * self.lookahead_min + perp_y * offset
                    if not self.check_line_collision(0.0, 0.0, ax, ay):
                        best_base_pt = (ax, ay)
                        break
                    offset += sign * self.avoidance_sample_step
                if best_base_pt is not None:
                    break

        if best_base_pt is not None:
            # 转回 map 坐标系
            ax_map = cx + cos_yaw * best_base_pt[0] - sin_yaw * best_base_pt[1]
            ay_map = cy + sin_yaw * best_base_pt[0] + cos_yaw * best_base_pt[1]
            best_map_pt = (ax_map, ay_map, cz)

            waypoint1 = (cx + (ax_map - cx) * 0.5, cy + (ay_map - cy) * 0.5, cz)
            reconnect_pt = self.reconnect_to_global(best_map_pt, fwd_x, fwd_y, cos_yaw, sin_yaw)
            
            if reconnect_pt is not None:
                self.avoidance_path = [waypoint1, best_map_pt, reconnect_pt]
            else:
                self.avoidance_path = [waypoint1, best_map_pt]
            return best_map_pt

        self.avoidance_path = None
        return orig_lookahead

    def reconnect_to_global(self, avoidance_pt_map, base_fwd_x, base_fwd_y, cos_yaw, sin_yaw):
        if self.global_path is None or len(self.global_path.poses) < 2:
            return None
            
        ax, ay, _ = avoidance_pt_map
        # 将 fwd 向量转回 map
        fwd_map_x = cos_yaw * base_fwd_x - sin_yaw * base_fwd_y
        fwd_map_y = sin_yaw * base_fwd_x + cos_yaw * base_fwd_y

        best_pt = None
        best_dist = float("inf")
        for pose_stamped in self.global_path.poses:
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            proj = (px - ax) * fwd_map_x + (py - ay) * fwd_map_y
            if proj > 0:
                d = math.hypot(px - ax, py - ay)
                if d < best_dist and d > self.lookahead_min:
                    best_dist = d
                    best_pt = (px, py, pose_stamped.pose.position.z)

        return best_pt

    def find_lookahead_point(self):
        if self.global_path is None or len(self.global_path.poses) == 0:
            return None
        if self.current_pose is None:
            return None

        cx, cy, _ = self.current_pose
        lookahead = self.lookahead_max

        closest_idx = 0
        closest_dist = float("inf")
        for i, pose_stamped in enumerate(self.global_path.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            d = self.distance_xy((cx, cy), (px, py))
            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        best_idx = closest_idx
        for i in range(closest_idx, len(self.global_path.poses)):
            px = self.global_path.poses[i].pose.position.x
            py = self.global_path.poses[i].pose.position.y
            if self.distance_xy((cx, cy), (px, py)) >= lookahead:
                best_idx = i
                break
        else:
            best_idx = len(self.global_path.poses) - 1

        goal_pt = self.global_path.poses[best_idx].pose.position
        return (goal_pt.x, goal_pt.y, goal_pt.z)

    def plan_loop(self):
        if not self.update_current_pose():
            return

        if self.global_path is None or len(self.global_path.poses) == 0:
            return

        orig_lookahead_pt = self.find_lookahead_point()
        if orig_lookahead_pt is None:
            return

        # 执行避障采样，更新前视点与 avoidance_path
        self.find_avoidance_lookahead(orig_lookahead_pt)
        self.publish_local_path()

    def publish_local_path(self):
        cx, cy, cz = self.current_pose

        if self.avoidance_path is not None:
            source_poses = self.avoidance_path
        else:
            source_poses = self.global_path.poses

        closest_idx = 0
        closest_dist = float("inf")
        for i, ps in enumerate(source_poses):
            if isinstance(ps, PoseStamped):
                px = ps.pose.position.x
                py = ps.pose.position.y
            else:
                px, py, _ = ps
            d = math.hypot(cx - px, cy - py)
            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        local_poses = []
        accumulated_dist = 0.0
        prev_pt = (cx, cy)

        # 始终将当前位置作为局部路径起点，保证曲线连续性
        start_pose = PoseStamped()
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.header.frame_id = self.map_frame
        start_pose.pose.position.x = cx
        start_pose.pose.position.y = cy
        start_pose.pose.position.z = cz
        local_poses.append(start_pose)

        for i in range(closest_idx, len(source_poses)):
            if isinstance(source_poses[i], PoseStamped):
                ppo = source_poses[i].pose.position
                pt = (ppo.x, ppo.y, ppo.z)
            else:
                pt = (source_poses[i][0], source_poses[i][1], source_poses[i][2])

            accumulated_dist += math.hypot(pt[0] - prev_pt[0], pt[1] - prev_pt[1])
            if i > closest_idx and accumulated_dist > self.local_path_lookahead:
                break

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = pt[2]
            local_poses.append(pose)
            prev_pt = pt

        if len(local_poses) < 2:
            return

        local_path_msg = Path()
        local_path_msg.header.stamp = self.get_clock().now().to_msg()
        local_path_msg.header.frame_id = self.map_frame
        local_path_msg.poses = local_poses

        self.pub_local_path.publish(local_path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NewLocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
