import rclpy
import heapq
import math
import tf2_ros
import numpy as np
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from custom_motion_plan_msgs.action import SendGoal

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('global_path_planner')
        
        # Parameters
        self.declare_parameter('layer_name', 'elevation')
        self.declare_parameter('slope_layer_name', 'slope')
        self.declare_parameter('traversability_layer_name', 'traversability')
        self.declare_parameter('max_elevation', 1.0)  # Max absolute height threshold
        self.declare_parameter('max_slope_angle', 45.0)  # Max slope angle in degrees
        self.declare_parameter('treat_nan_as_obstacle', True)
        self.declare_parameter('use_tf_for_start', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('path_z_offset', 0.05)  # Slight offset to prevent Z-fighting in RViz
        self.declare_parameter('replan_check_period', 1.0)   # 检查路径是否失效的周期(秒)
        self.declare_parameter('goal_tolerance', 0.5)  # 目标点到达容差(米)
        self.declare_parameter('path_check_lookahead', 5.0) # 向前检查路径失效的距离(米)
        self.declare_parameter('robot_radius', 3.0)      # 机器人半径
        
        self.layer_name = self.get_parameter('layer_name').value
        self.slope_layer_name = self.get_parameter('slope_layer_name').value
        self.trav_layer_name = self.get_parameter('traversability_layer_name').value
        self.max_elevation = self.get_parameter('max_elevation').value
        self.max_slope_angle = self.get_parameter('max_slope_angle').value
        self.treat_nan_as_obstacle = self.get_parameter('treat_nan_as_obstacle').value
        self.use_tf_for_start = self.get_parameter('use_tf_for_start').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.path_z_offset = self.get_parameter('path_z_offset').value
        self.replan_check_period = self.get_parameter('replan_check_period').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.path_check_lookahead = self.get_parameter('path_check_lookahead').value
        self.robot_radius = self.get_parameter('robot_radius').value

        # State
        self.grid_map = None
        self.map_data = None # elevation
        self.slope_data = None # slope
        self.trav_data = None # traversability
        self.map_info = None
        self.start_pose = None
        self.goal_pose = None
        self.first_path_msg = None # 第一次规划出来的路径
        self.is_navigating = False # 是否处于导航/重规划状态
        
        # 初始位置获取方式，从TF或/initialpose话题获取
        if self.use_tf_for_start:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.timer = self.create_timer(1.0, self.tf_start_pose_callback)
        else:
            self.sub_initial_pose = self.create_subscription(
                PoseWithCovarianceStamped,
                '/initialpose',
                self.initial_pose_callback,
                10)
            
        # Subscribers and Publishers
        self.sub_grid_map = self.create_subscription(
            GridMap,
            '/grid_map',
            self.grid_map_callback,
            1)
            
        self.sub_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
            
        self.pub_path = self.create_publisher(Path, '/global_path', 10)
        
        # Cache for continuous publishing
        self.current_path_msg = None
        self.path_pub_timer = self.create_timer(0.5, self.path_pub_timer_callback) # Publish at 2Hz

        # 事件驱动重规划定时器（仅用于检查路径有效性）
        self.replan_check_timer = self.create_timer(self.replan_check_period, self.check_path_validity_callback)

        # Action Server 接收目标点
        self.action_cb_group = ReentrantCallbackGroup()
        self.action_server = ActionServer(
            self,
            SendGoal,
            '/goal_check',
            self.execute_callback,
            callback_group=self.action_cb_group
        )

        self.get_logger().info("A* 全局路径规划器初始化完成 (事件驱动重规划)")

    def check_path_validity_callback(self):
        if not self.is_navigating or self.goal_pose is None or self.start_pose is None:
            return
            
        # 1. 检查是否到达目标点
        dx = self.start_pose.pose.position.x - self.goal_pose.pose.position.x
        dy = self.start_pose.pose.position.y - self.goal_pose.pose.position.y
        if math.hypot(dx, dy) < self.goal_tolerance:
            self.get_logger().info("全局规划器：已到达目标点附近，停止导航。")
            self.is_navigating = False
            self.current_path_msg = None
            return

        # 2. 如果当前没有有效路径，触发重规划
        if self.current_path_msg is None or len(self.current_path_msg.poses) == 0:
            self.get_logger().warn("全局规划器：当前无有效路径，触发重规划！")
            self.plan_path()
            return
            
        # 3. 检查当前已有路径在前方一段距离内是否被障碍物阻挡 (事件驱动核心)
        if self.is_path_blocked():
            self.get_logger().warn("全局规划器：检测到前方路径被障碍物阻挡，触发事件驱动重规划！")
            self.plan_path()

    def is_path_blocked(self):
        """检查当前保存的全局路径是否在最新的地图上变得不可通行"""
        if self.current_path_msg is None or self.map_data is None:
            return False
            
        curr_x = self.start_pose.pose.position.x
        curr_y = self.start_pose.pose.position.y
        
        # 找到机器人当前在路径上的最近点
        closest_idx = 0
        min_dist = float('inf')
        for i, pose_stamped in enumerate(self.current_path_msg.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            dist = math.hypot(px - curr_x, py - curr_y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        # 从最近点开始，向前方检查一定距离 (path_check_lookahead)
        accumulated_dist = 0.0
        for i in range(closest_idx, len(self.current_path_msg.poses)):
            px = self.current_path_msg.poses[i].pose.position.x
            py = self.current_path_msg.poses[i].pose.position.y
            
            if i > closest_idx:
                prev_x = self.current_path_msg.poses[i-1].pose.position.x
                prev_y = self.current_path_msg.poses[i-1].pose.position.y
                accumulated_dist += math.hypot(px - prev_x, py - prev_y)
                
            # 只检查前方有限距离内的路况
            if accumulated_dist > self.path_check_lookahead:
                break
                
            # 将路径点坐标转为地图索引
            idx = self.position_to_index(px, py)
            if idx is None or not self.is_valid_index(idx[0], idx[1]):
                continue
                
            # 检查该点在最新地图上是否变成了障碍物 (考虑高程和通行度)
            if self.is_obstacle(idx[0], idx[1]):
                return True
                
        return False

    def path_pub_timer_callback(self):
        if self.current_path_msg is not None:
            # Update timestamp to prevent RViz from fading/discarding old paths
            self.current_path_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_path.publish(self.current_path_msg)

    def tf_start_pose_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            
            if self.start_pose is None:
                self.get_logger().info(f"从 TF 获取初始起始位姿 ({self.base_frame}): x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
            self.start_pose = pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"等待 TF 从 {self.map_frame} 到 {self.base_frame}...", throttle_duration_sec=5.0)

    def initial_pose_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.start_pose = pose
        self.get_logger().info(f"已接收初始起始位姿: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.is_navigating = True
        self.get_logger().info(f"已接收目标位姿: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.plan_path()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Action Server 收到目标点请求...')
        
        # 1. 接收目标点并设置
        self.goal_pose = goal_handle.request.goal_pose
        self.is_navigating = True
        
        initial_distance = 0.0
        if self.start_pose and self.goal_pose:
            dx = self.start_pose.pose.position.x - self.goal_pose.pose.position.x
            dy = self.start_pose.pose.position.y - self.goal_pose.pose.position.y
            initial_distance = math.hypot(dx, dy)

        # 发送初始状态反馈
        feedback_msg = SendGoal.Feedback()
        feedback_msg.current_time = self.get_clock().now().to_msg()
        feedback_msg.current_stage = 1 # STAGE_GLOBAL_PLANNING
        goal_handle.publish_feedback(feedback_msg)
        
        # 2. 触发全局规划
        success, error_code, msg = self.plan_path()
        if not success:
            goal_handle.abort()
            result = SendGoal.Result()
            result.success = False
            result.error_code = error_code
            result.message = msg
            result.finish_time = self.get_clock().now().to_msg()
            return result
            
        self.get_logger().info('全局规划成功，开始监控到达状态...')
        
        # 3. 循环等待直到到达目标点 (在 check_path_validity_callback 中到达后会设置 is_navigating=False)
        while rclpy.ok() and self.is_navigating:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.is_navigating = False
                self.current_path_msg = None
                self.get_logger().info('目标点请求被取消')
                result = SendGoal.Result()
                result.success = False
                result.error_code = 2 # 超时或被取消
                result.message = "目标已取消"
                result.finish_time = self.get_clock().now().to_msg()
                return result
                
            # 填充 feedback
            if self.start_pose and self.goal_pose:
                dx = self.start_pose.pose.position.x - self.goal_pose.pose.position.x
                dy = self.start_pose.pose.position.y - self.goal_pose.pose.position.y
                dist_rem = math.hypot(dx, dy)
                
                feedback_msg.current_time = self.get_clock().now().to_msg()
                feedback_msg.current_stage = 2 # STAGE_MOVING
                feedback_msg.distance_remaining = float(dist_rem)
                if initial_distance > 0:
                    ratio = (initial_distance - dist_rem) / initial_distance
                    feedback_msg.completion_ratio = float(max(0.0, min(1.0, ratio)))
                else:
                    feedback_msg.completion_ratio = 1.0
                    
                goal_handle.publish_feedback(feedback_msg)
                
            time.sleep(0.5)
            
        # 成功到达目标点
        goal_handle.succeed()
        result = SendGoal.Result()
        result.success = True
        result.error_code = 0
        result.message = "成功到达目标点"
        result.finish_time = self.get_clock().now().to_msg()
        return result

    def grid_map_callback(self, msg):
        try:
            layer_idx = msg.layers.index(self.layer_name)
            slope_idx = msg.layers.index(self.slope_layer_name)
            trav_idx = msg.layers.index(self.trav_layer_name)
        except ValueError:
            self.get_logger().warn(f"在 GridMap 中未找到所需层 ({self.layer_name}, {self.slope_layer_name}, {self.trav_layer_name})", throttle_duration_sec=5.0)
            return
            
        self.map_info = msg.info
        
        # Extract data
        multi_array = msg.data[layer_idx]
        slope_array = msg.data[slope_idx]
        trav_array = msg.data[trav_idx]
        
        # Use layout dimensions if available
        size_x = int(round(self.map_info.length_x / self.map_info.resolution))
        size_y = int(round(self.map_info.length_y / self.map_info.resolution))
        
        if len(multi_array.layout.dim) >= 2:
            for dim in multi_array.layout.dim:
                if dim.label == 'column_index':
                    size_y = dim.size
                elif dim.label == 'row_index':
                    size_x = dim.size
        
        if len(multi_array.data) != size_x * size_y:
            self.get_logger().warn(f"GridMap 数据大小不匹配。预期大小： {size_x * size_y}, 实际大小： {len(multi_array.data)}")
            return
            
        # GridMap stores data in column-major order (Eigen default)
        # Using order='F' (Fortran-like) ensures X (rows) varies first, then Y (cols)
        self.map_data = np.array(multi_array.data, dtype=np.float32).reshape((size_x, size_y), order='F')
        self.slope_data = np.array(slope_array.data, dtype=np.float32).reshape((size_x, size_y), order='F')
        self.trav_data = np.array(trav_array.data, dtype=np.float32).reshape((size_x, size_y), order='F')
        
    def position_to_index(self, x, y):
        if self.map_info is None:
            return None
        res = self.map_info.resolution
        center_x = self.map_info.pose.position.x
        center_y = self.map_info.pose.position.y
        length_x = self.map_info.length_x
        length_y = self.map_info.length_y
        
        max_x = center_x + length_x / 2.0
        max_y = center_y + length_y / 2.0
        
        idx_x = int((max_x - x) / res)
        idx_y = int((max_y - y) / res)
        
        return (idx_x, idx_y)
        
    def index_to_position(self, idx_x, idx_y):
        if self.map_info is None:
            return None
        res = self.map_info.resolution
        center_x = self.map_info.pose.position.x
        center_y = self.map_info.pose.position.y
        length_x = self.map_info.length_x
        length_y = self.map_info.length_y
        
        max_x = center_x + length_x / 2.0
        max_y = center_y + length_y / 2.0
        
        x = max_x - (idx_x + 0.5) * res
        y = max_y - (idx_y + 0.5) * res
        
        return (x, y)
        
    def is_valid_index(self, idx_x, idx_y):
        if self.map_data is None:
            return False
        return 0 <= idx_x < self.map_data.shape[0] and 0 <= idx_y < self.map_data.shape[1]
        
    def is_obstacle(self, idx_x, idx_y):
        # 检查高程层是否异常
        val = self.map_data[idx_x, idx_y]
        if np.isnan(val):
            return self.treat_nan_as_obstacle
        if val > self.max_elevation:
            return True
            
        # 检查通行度层 (traversability == 0.0 表示不可通行)
        if self.trav_data is not None:
            trav = self.trav_data[idx_x, idx_y]
            if np.isnan(trav) or trav <= 0.01:
                return True
                
        return False

    def is_valid_move(self, curr_idx, next_idx):
        # 既然我们已经有了提前计算好的通行度层，这里只需验证通行度即可
        # 如果通行度为 0，is_obstacle 已经被拦截了，这里可以做更精细的判断
        
        val_curr = self.map_data[curr_idx[0], curr_idx[1]]
        val_next = self.map_data[next_idx[0], next_idx[1]]
        
        if np.isnan(val_curr) or np.isnan(val_next):
            return not self.treat_nan_as_obstacle
            
        dz = abs(val_next - val_curr)
        dx = next_idx[0] - curr_idx[0]
        dy = next_idx[1] - curr_idx[1]
        d_grid = math.hypot(dx, dy)
        d_meters = d_grid * self.map_info.resolution
        
        if d_meters == 0:
            return True
            
        slope_angle = math.degrees(math.atan2(dz, d_meters))
        if slope_angle > self.max_slope_angle:
            return False
            
        # 使用通行度层判断
        if self.trav_data is not None:
            trav_next = self.trav_data[next_idx[0], next_idx[1]]
            if np.isnan(trav_next) or trav_next <= 0.05:  # 设定一个极小的通行度阈值
                return False
            
        return True

    def heuristic(self, a_idx, b_idx):
        # 3D Euclidean distance heuristic
        # 节点 A 的 Z 坐标值
        a_z = self.map_data[a_idx[0], a_idx[1]]
        # 节点 B 的 Z 坐标值
        b_z = self.map_data[b_idx[0], b_idx[1]]
        
        if np.isnan(a_z): a_z = 0.0
        if np.isnan(b_z): b_z = 0.0
        
        d_grid = math.hypot(a_idx[0] - b_idx[0], a_idx[1] - b_idx[1])
        d_meters = d_grid * self.map_info.resolution
        dz = abs(a_z - b_z)
        
        return math.hypot(d_meters, dz)

    def plan_path(self):
        if self.map_data is None:
            msg = "无法进行全局路径规划：地图数据未接收！"
            self.get_logger().warn(msg)
            return False, 1, msg
        if self.start_pose is None:
            msg = "无法进行全局路径规划：起始点未接收！"
            self.get_logger().warn(msg)
            return False, 1, msg
        if self.goal_pose is None:
            msg = "无法进行全局路径规划：目标点未接收！"
            self.get_logger().warn(msg)
            return False, 1, msg
            
        start_x = self.start_pose.pose.position.x
        start_y = self.start_pose.pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        
        start_idx = self.position_to_index(start_x, start_y)
        goal_idx = self.position_to_index(goal_x, goal_y)
        
        if not self.is_valid_index(*start_idx):
            msg = "无法进行全局路径规划：起始点超出地图范围！"
            self.get_logger().warn(msg)
            return False, 1, msg
        if not self.is_valid_index(*goal_idx):
            msg = "无法进行全局路径规划：目标点超出地图范围！"
            self.get_logger().warn(msg)
            return False, 1, msg
            
        if self.is_obstacle(*start_idx):
            msg = f"无法进行全局路径规划：起始点在障碍物或高度过高！(Z={self.map_data[start_idx[0], start_idx[1]]:.2f})"
            self.get_logger().warn(msg)
            return False, 1, msg
            
        if self.is_obstacle(*goal_idx):
            msg = f"无法进行全局路径规划：目标点存在障碍物或高度过高！(Z={self.map_data[goal_idx[0], goal_idx[1]]:.2f})"
            self.get_logger().warn(msg)
            return False, 1, msg
            
        self.get_logger().info(f"开始全局路径规划：从{start_idx} 到{goal_idx}...")
        
        start_time = self.get_clock().now()
        
        # A* algorithm
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        
        frontier = []
        heapq.heappush(frontier, (0, start_idx))
        
        came_from = {}
        cost_so_far = {}
        
        came_from[start_idx] = None
        cost_so_far[start_idx] = 0.0
        
        path_found = False
        
        while frontier:
            _, current = heapq.heappop(frontier)
            
            if current == goal_idx:
                path_found = True
                break
                
            for dx, dy in neighbors:
                next_idx = (current[0] + dx, current[1] + dy)
                
                if not self.is_valid_index(*next_idx):
                    continue
                if self.is_obstacle(*next_idx):
                    continue
                if not self.is_valid_move(current, next_idx):
                    continue
                    
                # Calculate 3D move cost
                val_curr = self.map_data[current[0], current[1]]
                val_next = self.map_data[next_idx[0], next_idx[1]]
                
                dz = abs(val_next - val_curr) if not (np.isnan(val_curr) or np.isnan(val_next)) else 0.0
                d_grid = math.hypot(dx, dy)
                d_meters = d_grid * self.map_info.resolution
                
                # 基础几何移动代价
                base_move_cost = math.hypot(d_meters, dz)
                
                # 结合通行度层（Traversability Layer）计算惩罚代价
                # traversability 范围是 [0, 1], 1 表示平坦，0 表示不可通行
                if self.trav_data is not None:
                    trav_next = self.trav_data[next_idx[0], next_idx[1]]
                    # 当 trav_next 接近 0 时，惩罚增大
                    penalty_weight = 2.0  # 惩罚权重，可根据需要调整
                    trav_penalty = 1.0 + penalty_weight * (1.0 - max(0.01, float(trav_next) if not np.isnan(trav_next) else 0.01))
                    move_cost = base_move_cost * trav_penalty
                else:
                    move_cost = base_move_cost
                
                new_cost = cost_so_far[current] + move_cost
                
                if next_idx not in cost_so_far or new_cost < cost_so_far[next_idx]:
                    cost_so_far[next_idx] = new_cost
                    priority = new_cost + self.heuristic(next_idx, goal_idx)
                    heapq.heappush(frontier, (priority, next_idx))
                    came_from[next_idx] = current
                    
        if not path_found:
            end_time = self.get_clock().now()
            duration = (end_time - start_time).nanoseconds / 1e9
            msg = f"A* 算法寻找路径失败！目标点由于坡度或高度约束而不可达！耗时: {duration:.4f} 秒"
            self.get_logger().warn(msg)
            return False, 1, msg
            
        # Reconstruct path
        current = goal_idx
        path_indices = []
        while current is not None:
            path_indices.append(current)
            current = came_from[current]
        path_indices.reverse()
        
        # Smooth the path using Bezier curve
        smoothed_path = self.smooth_path_bezier(path_indices)
        
        end_time = self.get_clock().now()
        duration = (end_time - start_time).nanoseconds / 1e9
        msg = f"全局路径规划计算完成，耗时: {duration:.4f} 秒"
        self.get_logger().info(msg)
        
        self.publish_path(smoothed_path)
        return True, 0, msg
        
    def smooth_path_bezier(self, path_indices, num_points=100):
        """Smooth the generated path using a 3rd order Bezier curve (Cubic Bezier)."""
        if len(path_indices) < 2:
            return path_indices
            
        # 如果路径点少于4个，我们可以通过插值补充，或者直接退化为简单的连线/低阶贝塞尔
        # 这里为了确保是三阶贝塞尔，我们从原路径中提取4个控制点：起点、1/3点、2/3点、终点
        p0 = path_indices[0]
        p3 = path_indices[-1]
        
        if len(path_indices) == 2:
            # 只有两个点，控制点均匀分布在两点连线上
            p1 = (p0[0] + (p3[0]-p0[0])/3.0, p0[1] + (p3[1]-p0[1])/3.0)
            p2 = (p0[0] + 2.0*(p3[0]-p0[0])/3.0, p0[1] + 2.0*(p3[1]-p0[1])/3.0)
        elif len(path_indices) == 3:
            p1 = path_indices[1]
            p2 = path_indices[1] # 复用中间点
        else:
            idx1 = len(path_indices) // 3
            idx2 = 2 * len(path_indices) // 3
            p1 = path_indices[idx1]
            p2 = path_indices[idx2]

        smoothed = []
        for i in range(num_points):
            t = i / float(num_points - 1)
            
            # 三阶贝塞尔曲线公式: B(t) = (1-t)^3 * P0 + 3(1-t)^2 * t * P1 + 3(1-t) * t^2 * P2 + t^3 * P3
            x = ((1 - t)**3 * p0[0] + 
                 3 * (1 - t)**2 * t * p1[0] + 
                 3 * (1 - t) * t**2 * p2[0] + 
                 t**3 * p3[0])
                 
            y = ((1 - t)**3 * p0[1] + 
                 3 * (1 - t)**2 * t * p1[1] + 
                 3 * (1 - t) * t**2 * p2[1] + 
                 t**3 * p3[1])
                 
            smoothed.append([x, y])
            
        return smoothed

    def yaw_to_quaternion(self, yaw):
        """Convert a yaw angle to a geometry_msgs Quaternion"""
        q = PoseStamped().pose.orientation
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def publish_path(self, path_indices):
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Pre-compute positions
        positions = []
        for idx in path_indices:
            # path_indices might be float due to smoothing, so interpolate position
            # using the base logic:
            x, y = self.index_to_position(idx[0], idx[1])
            
            # Fetch valid Z from map data by using the nearest integer index
            int_idx_x, int_idx_y = int(round(idx[0])), int(round(idx[1]))
            # Bound check for integer indices
            int_idx_x = max(0, min(int_idx_x, self.map_data.shape[0] - 1))
            int_idx_y = max(0, min(int_idx_y, self.map_data.shape[1] - 1))
            
            val = self.map_data[int_idx_x, int_idx_y]
            z = float(val) + self.path_z_offset if not np.isnan(val) else self.path_z_offset
            positions.append((float(x), float(y), float(z)))
        
        for i, (x, y, z) in enumerate(positions):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            
            # Calculate orientation based on path direction
            if i < len(positions) - 1:
                # Point towards the next node
                next_x, next_y, _ = positions[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
                pose.pose.orientation = self.yaw_to_quaternion(yaw)
            else:
                # Last node: Use the exact orientation from the goal pose
                if self.goal_pose is not None:
                    pose.pose.orientation = self.goal_pose.pose.orientation
                else:
                    pose.pose.orientation.w = 1.0
                    
            path_msg.poses.append(pose)
            
        self.current_path_msg = path_msg
        self.get_logger().info(f"已发布全局路径，包含{len(path_msg.poses)}个点")
        self.pub_path.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    
    # 使用多线程执行器，以便Action Server中的time.sleep不会阻塞定时器和回调
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()