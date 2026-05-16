import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped
import tf2_ros
import math
import numpy as np

class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__("local_path_planner")

        # Parameters
        self.declare_parameter("layer_name", "elevation")
        self.declare_parameter("max_slope_angle", 45.0)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("max_elevation", 1.0)

        # Pure Pursuit parameters
        self.declare_parameter("lookahead_min", 0.3)
        self.declare_parameter("lookahead_max", 2.0)
        self.declare_parameter("lookahead_scale", 0.3)
        self.declare_parameter("wheel_base", 0.3)
        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("goal_tolerance", 0.15)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("local_path_lookahead", 3.0)

        self.declare_parameter("lidar_topics", ["/lidar3_points"])
        self.declare_parameter("obstacle_safety_distance", 0.3)
        self.declare_parameter("obstacle_check_ahead", 3.0)
        self.declare_parameter("local_grid_resolution", 0.1)
        self.declare_parameter("local_grid_size", 40)
        self.declare_parameter("max_lateral_offset", 1.5)
        self.declare_parameter("avoidance_sample_step", 0.1)

        self.layer_name = self.get_parameter("layer_name").value
        self.max_slope_angle = self.get_parameter("max_slope_angle").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.max_elevation = self.get_parameter("max_elevation").value

        self.lookahead_min = self.get_parameter("lookahead_min").value
        self.lookahead_max = self.get_parameter("lookahead_max").value
        self.lookahead_scale = self.get_parameter("lookahead_scale").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.control_rate = self.get_parameter("control_rate").value
        self.local_path_lookahead = self.get_parameter("local_path_lookahead").value

        self.lidar_topics = self.get_parameter("lidar_topics").value
        self.obstacle_safety_distance = self.get_parameter("obstacle_safety_distance").value
        self.obstacle_check_ahead = self.get_parameter("obstacle_check_ahead").value
        self.local_grid_resolution = self.get_parameter("local_grid_resolution").value
        self.local_grid_size = self.get_parameter("local_grid_size").value
        self.max_lateral_offset = self.get_parameter("max_lateral_offset").value
        self.avoidance_sample_step = self.get_parameter("avoidance_sample_step").value

        # State
        self.global_path = None
        self.map_data = None
        self.map_info = None
        self.current_pose = None
        self.current_yaw = 0.0
        self.goal_reached = False
        self.avoidance_path = None
        self.latest_cloud_msg = None
        self.lidar_timestamp = None

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.sub_global_path = self.create_subscription(
            Path, "/global_path", self.global_path_callback, 10
        )
        self.sub_grid_map = self.create_subscription(
            GridMap, "/grid_map", self.grid_map_callback, 1
        )
        self.sub_lidar_points = self.create_subscription(
            PointCloud2, "/lidar3_points", self.lidar_points_callback, 10
        )

        # Publisher: velocity commands (stamped with timestamp)
        self.pub_cmd_vel = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # 局部路径发布
        self.local_path_pub = self.create_publisher(
            Path,
            "/local_path",
            10,
        )
        # 创建定时器，循环发布局部路径
        self.pub_local_path_timer = self.create_timer(
            0.5,
            self.publish_local_path_callback
        )

        # Control loop timer
        period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info("局部路径规划器初始化完成 (Pure Pursuit + 动态避障)")

        

    # ---------- Callbacks ----------
    def publish_local_path_callback(self):
        if self.global_path is None or len(self.global_path.poses) == 0:
            return
        if self.current_pose is None:
            return

        cx, cy, _ = self.current_pose

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

        for i in range(closest_idx, len(source_poses)):
            if isinstance(source_poses[i], PoseStamped):
                ppo = source_poses[i].pose.position
                pt = (ppo.x, ppo.y)
            else:
                pt = (source_poses[i][0], source_poses[i][1])

            accumulated_dist += math.hypot(pt[0] - prev_pt[0], pt[1] - prev_pt[1])
            if i > closest_idx and accumulated_dist > self.local_path_lookahead:
                break

            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            z = self.get_path_point_z(pt[0], pt[1])
            pose.pose.position.z = z
            local_poses.append(pose)
            prev_pt = pt

        if len(local_poses) < 2:
            return

        local_path_msg = Path()
        local_path_msg.header.stamp = self.get_clock().now().to_msg()
        local_path_msg.header.frame_id = self.map_frame
        local_path_msg.poses = local_poses

        self.local_path_pub.publish(local_path_msg)

    def lidar_points_callback(self, msg):
        self.latest_cloud_msg = msg
        self.lidar_timestamp = self.get_clock().now()

    def cloud_to_obstacle_grid(self):
        if self.latest_cloud_msg is None or self.lidar_timestamp is None:
            return None
        if self.current_pose is None:
            return None

        age = (self.get_clock().now() - self.lidar_timestamp).nanoseconds / 1e9
        if age > 0.5:
            return None

        try:
            t_base_map = self.tf_buffer.lookup_transform(
                self.base_frame, self.latest_cloud_msg.header.frame_id, rclpy.time.Time()
            )
        except Exception:
            return None

        pts = []
        offset_to_first = 0
        x_idx = y_idx = z_idx = -1
        for field in self.latest_cloud_msg.fields:
            if field.name == "x":
                x_idx = offset_to_first
            elif field.name == "y":
                y_idx = offset_to_first
            elif field.name == "z":
                z_idx = offset_to_first
            offset_to_first += 1

        if x_idx < 0 or y_idx < 0 or z_idx < 0:
            return None

        data = np.frombuffer(self.latest_cloud_msg.data, dtype=np.float32)
        point_step_floats = self.latest_cloud_msg.point_step // 4
        num_points = self.latest_cloud_msg.width * self.latest_cloud_msg.height
        if len(data) < num_points * point_step_floats:
            return None

        pc = data.reshape((num_points, point_step_floats))
        x_arr = pc[:, x_idx]
        y_arr = pc[:, y_idx]
        z_arr = pc[:, z_idx]
        valid = np.isfinite(x_arr) & np.isfinite(y_arr) & np.isfinite(z_arr)
        x_arr = x_arr[valid]
        y_arr = y_arr[valid]
        z_arr = z_arr[valid]

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
        pts_lidar = np.column_stack((x_arr, y_arr, z_arr))
        pts_base = (R @ pts_lidar.T).T + np.array([tx, ty, tz])

        half_size = (self.local_grid_size * self.local_grid_resolution) / 2.0
        mask_x = np.abs(pts_base[:, 0]) <= half_size
        mask_y = np.abs(pts_base[:, 1]) <= half_size
        mask = mask_x & mask_y

        height_thresh = self.obstacle_safety_distance * 0.3
        mask_z = np.abs(pts_base[:, 2]) <= height_thresh
        mask = mask & mask_z

        pts_base = pts_base[mask]
        if len(pts_base) == 0:
            return np.zeros((self.local_grid_size, self.local_grid_size), dtype=np.uint8)

        grid = np.zeros((self.local_grid_size, self.local_grid_size), dtype=np.uint8)
        ix = np.floor((pts_base[:, 0] + half_size) / self.local_grid_resolution).astype(int)
        iy = np.floor((pts_base[:, 1] + half_size) / self.local_grid_resolution).astype(int)
        valid = (ix >= 0) & (ix < self.local_grid_size) & (iy >= 0) & (iy < self.local_grid_size)
        grid[ix[valid], iy[valid]] = 1

        safety_cells = int(np.ceil(self.obstacle_safety_distance / self.local_grid_resolution))
        if safety_cells > 0:
            dilated = np.copy(grid)
            for dx in range(-safety_cells, safety_cells + 1):
                for dy in range(-safety_cells, safety_cells + 1):
                    if dx == 0 and dy == 0:
                        continue
                    shifted = np.roll(grid, (dx, dy), axis=(0, 1))
                    if dx > 0:
                        shifted[:dx, :] = 0
                    elif dx < 0:
                        shifted[dx:, :] = 0
                    if dy > 0:
                        shifted[:, :dy] = 0
                    elif dy < 0:
                        shifted[:, dy:] = 0
                    dilated |= shifted
            grid = dilated

        return grid

    def check_line_collision(self, x1, y1, x2, y2, grid):
        half = (self.local_grid_size * self.local_grid_resolution) / 2.0
        steps = max(1, int(math.hypot(x2 - x1, y2 - y1) / (self.local_grid_resolution * 0.5)))
        for s in range(steps + 1):
            t = s / max(steps, 1)
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            ix = int(np.floor((x + half) / self.local_grid_resolution))
            iy = int(np.floor((y + half) / self.local_grid_resolution))
            if 0 <= ix < self.local_grid_size and 0 <= iy < self.local_grid_size:
                if grid[ix, iy]:
                    return False
        return True

    def find_avoidance_lookahead(self, orig_lookahead):
        grid = self.cloud_to_obstacle_grid()
        if grid is None:
            self.avoidance_path = None
            return orig_lookahead

        cx, cy, cz = self.current_pose
        lx, ly, lz = orig_lookahead
        dx = lx - cx
        dy = ly - cy
        ld = math.hypot(dx, dy)
        if ld < 1e-6:
            return orig_lookahead

        fwd_x = dx / ld
        fwd_y = dy / ld
        perp_x = -fwd_y
        perp_y = fwd_x

        if self.check_line_collision(0.0, 0.0, ld, 0.0, grid):
            self.avoidance_path = None
            return orig_lookahead

        half = (self.local_grid_size * self.local_grid_resolution) / 2.0
        best = None
        best_score = float("inf")
        check_dist = min(ld, self.obstacle_check_ahead)

        for sign in (-1.0, 1.0):
            offset = sign * self.avoidance_sample_step
            while abs(offset) <= self.max_lateral_offset:
                ax = fwd_x * check_dist + perp_x * offset
                ay = fwd_y * check_dist + perp_y * offset

                if not (-half < ax < half and -half < ay < half):
                    offset += sign * self.avoidance_sample_step
                    continue

                if self.check_line_collision(0.0, 0.0, ax, ay, grid):
                    score = abs(offset) + math.hypot(ax - check_dist, ay)
                    if score < best_score:
                        best_score = score
                        best = (cx + ax, cy + ay, cz)
                    break
                offset += sign * self.avoidance_sample_step

        if best is None:
            self.get_logger().warn("避障采样未找到可行路径，减速并尝试小步避障", throttle_duration_sec=1.0)

            for sign in (-1.0, 1.0):
                offset = sign * self.avoidance_sample_step
                while abs(offset) <= self.max_lateral_offset * 0.5:
                    ax = fwd_x * self.lookahead_min + perp_x * offset
                    ay = fwd_y * self.lookahead_min + perp_y * offset
                    if self.check_line_collision(0.0, 0.0, ax, ay, grid):
                        best = (cx + ax, cy + ay, cz)
                        break
                    offset += sign * self.avoidance_sample_step
                if best is not None:
                    break

        if best is not None:
            waypoint1 = (cx + perp_x * (best[0] - cx) * 0.5, cy + perp_y * (best[1] - cy) * 0.5, cz)
            waypoint2 = best
            reconnect_pt = self.reconnect_to_global(best, fwd_x, fwd_y)
            if reconnect_pt is not None:
                self.avoidance_path = [waypoint1, waypoint2, reconnect_pt]
            else:
                self.avoidance_path = [waypoint1, waypoint2]
            return waypoint2

        self.avoidance_path = None
        return orig_lookahead

    def reconnect_to_global(self, avoidance_pt, fwd_x, fwd_y):
        if self.global_path is None or len(self.global_path.poses) < 2:
            return None
        ax, ay, _ = avoidance_pt

        best_pt = None
        best_dist = float("inf")
        for pose_stamped in self.global_path.poses:
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            proj = (px - ax) * fwd_x + (py - ay) * fwd_y
            if proj > 0:
                d = math.hypot(px - ax, py - ay)
                if d < best_dist and d > self.lookahead_min:
                    best_dist = d
                    best_pt = (px, py, pose_stamped.pose.position.z)

        return best_pt

    def global_path_callback(self, msg):
        # 判断是否为新的目标点，只有目标点发生明显变化时才重置 goal_reached 状态
        if self.global_path is not None and len(msg.poses) > 0 and len(self.global_path.poses) > 0:
            old_goal = self.global_path.poses[-1].pose.position
            new_goal = msg.poses[-1].pose.position
            dist = self.distance_xy((new_goal.x, new_goal.y), (old_goal.x, old_goal.y))
            if dist > 0.1:
                self.goal_reached = False
        else:
            self.goal_reached = False
            
        self.global_path = msg

    def grid_map_callback(self, msg):
        try:
            layer_idx = msg.layers.index(self.layer_name)
        except ValueError:
            return
        self.map_info = msg.info
        multi_array = msg.data[layer_idx]
        size_x = int(round(self.map_info.length_x / self.map_info.resolution))
        size_y = int(round(self.map_info.length_y / self.map_info.resolution))
        for dim in multi_array.layout.dim:
            if dim.label == "column_index":
                size_y = dim.size
            elif dim.label == "row_index":
                size_x = dim.size
        if len(multi_array.data) == size_x * size_y:
            self.map_data = np.array(multi_array.data, dtype=np.float32).reshape(
                (size_x, size_y), order="F"
            )

    # ---------- Helpers ----------

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
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return False

    def distance_xy(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def get_path_point_z(self, x, y):
        if self.map_data is None or self.map_info is None:
            return 0.0
        res = self.map_info.resolution
        center_x = self.map_info.pose.position.x
        center_y = self.map_info.pose.position.y
        length_x = self.map_info.length_x
        length_y = self.map_info.length_y
        max_x = center_x + length_x / 2.0
        max_y = center_y + length_y / 2.0
        ix = int((max_x - x) / res)
        iy = int((max_y - y) / res)
        if 0 <= ix < self.map_data.shape[0] and 0 <= iy < self.map_data.shape[1]:
            val = self.map_data[ix, iy]
            if not np.isnan(val):
                return float(val)
        return 0.0

    def check_local_obstacle(self, cx, cy, cz):
        if self.map_data is None or self.map_info is None:
            return False
        res = self.map_info.resolution
        center_x = self.map_info.pose.position.x
        center_y = self.map_info.pose.position.y
        length_x = self.map_info.length_x
        length_y = self.map_info.length_y
        max_x = center_x + length_x / 2.0
        max_y = center_y + length_y / 2.0
        ix = int((max_x - cx) / res)
        iy = int((max_y - cy) / res)
        if 0 <= ix < self.map_data.shape[0] and 0 <= iy < self.map_data.shape[1]:
            val = self.map_data[ix, iy]
            if np.isnan(val):
                return True
            if val > self.max_elevation:
                return True
            local_slope = math.degrees(math.atan2(abs(val - cz), res))
            if local_slope > self.max_slope_angle:
                return True
        return False

    # ---------- Pure Pursuit core ----------

    def find_lookahead_point(self):
        if self.global_path is None or len(self.global_path.poses) == 0:
            return None, -1
        if self.current_pose is None:
            return None, -1

        cx, cy, _ = self.current_pose

        # Adaptive lookahead: longer at higher speeds
        current_speed = 0.0
        lookahead = self.lookahead_min + self.lookahead_scale * current_speed
        lookahead = max(self.lookahead_min, min(self.lookahead_max, lookahead))

        # Find the closest point index on path
        closest_idx = 0
        closest_dist = float("inf")
        for i, pose_stamped in enumerate(self.global_path.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            d = self.distance_xy((cx, cy), (px, py))
            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        # Search forward for the first point beyond lookahead distance
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
        return (goal_pt.x, goal_pt.y, goal_pt.z), best_idx

    def compute_velocity(self, lookahead_pt, goal_idx):
        if self.current_pose is None:
            return 0.0, 0.0

        cx, cy, cz = self.current_pose

        avoided_pt = self.find_avoidance_lookahead(lookahead_pt)
        lx, ly, lz = avoided_pt

        dx = lx - cx
        dy = ly - cy
        ld = math.hypot(dx, dy)

        if ld < 1e-6:
            return 0.0, 0.0

        # Transform lookahead point into robot frame
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy

        # Pure Pursuit: curvature = 2 * lateral_error / Ld^2
        curvature = 2.0 * local_y / (ld * ld)
        curvature = max(-1.0, min(1.0, curvature))
        angular_z = curvature * self.max_linear_speed
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

        # Linear speed
        linear_x = self.max_linear_speed

        # Slow down on high curvature
        curve_factor = 1.0 - abs(curvature) * 0.8
        linear_x *= max(0.2, curve_factor)

        # Slow down near goal (last few points)
        if self.global_path is not None and goal_idx >= 0:
            remaining = len(self.global_path.poses) - goal_idx
            if remaining < 5:
                linear_x *= max(0.1, remaining / 5.0)

        # Obstacle check along forward direction
        forward_steps = 5
        step_size = ld / max(forward_steps, 1)
        for s in range(1, forward_steps + 1):
            sx = cx + (dx / ld) * step_size * s
            sy = cy + (dy / ld) * step_size * s
            sz = self.get_path_point_z(sx, sy)
            if self.check_local_obstacle(sx, sy, sz):
                self.get_logger().warn("检测到前方障碍物，减速！", throttle_duration_sec=2.0)
                linear_x = 0.0
                break

        # Slope check
        if self.map_data is not None:
            dz = abs(lz - cz)
            d_meters = ld
            if d_meters > 0:
                slope_angle = math.degrees(math.atan2(dz, d_meters))
                if slope_angle > self.max_slope_angle * 0.8:
                    linear_x *= 0.5

        # Goal reached check
        if self.global_path is not None and goal_idx >= len(self.global_path.poses) - 1:
            final_pt = self.global_path.poses[-1].pose.position
            if self.distance_xy((cx, cy), (final_pt.x, final_pt.y)) < self.goal_tolerance:
                if not self.goal_reached:
                    self.get_logger().info("已到达目标点！")
                    self.goal_reached = True
                return 0.0, 0.0

        return linear_x, angular_z

    def publish_zero_velocity(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.base_frame
        self.pub_cmd_vel.publish(cmd)

    # ---------- Main control loop ----------

    def control_loop(self):
        if not self.update_current_pose():
            self.publish_zero_velocity()
            return

        if self.global_path is None or len(self.global_path.poses) == 0:
            self.publish_zero_velocity()
            return

        lookahead_pt, goal_idx = self.find_lookahead_point()
        if lookahead_pt is None:
            self.publish_zero_velocity()
            return

        linear_x, angular_z = self.compute_velocity(lookahead_pt, goal_idx)

        if not self.goal_reached:
            cx, cy, cz = self.current_pose
            yaw_deg = math.degrees(self.current_yaw)
            self.get_logger().info(
                f"运行中 -> 位置: ({cx:.2f}, {cy:.2f}, {cz:.2f}), 朝向: {yaw_deg:.1f}度, 速度: v={linear_x:.2f} m/s, w={angular_z:.2f} rad/s",
                throttle_duration_sec=1.0
            )

        now = self.get_clock().now().to_msg()
        cmd = TwistStamped()
        cmd.header.stamp = now
        cmd.header.frame_id = self.base_frame
        cmd.twist.linear.x = linear_x
        cmd.twist.angular.z = angular_z
        self.pub_cmd_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
