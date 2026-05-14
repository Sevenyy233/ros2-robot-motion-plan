import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TwistStamped
from custom_motion_plan_msgs.msg import RobotTrajectory
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

        # State
        self.global_path = None
        self.map_data = None
        self.map_info = None
        self.current_pose = None
        self.current_yaw = 0.0
        self.goal_reached = False

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

        # Publisher: velocity commands (stamped with timestamp)
        self.pub_cmd_vel = self.create_publisher(TwistStamped, "/cmd_vel", 10)

        # 局部路径发布者
        self.local_path_pub = self.create_publisher(
            Path,
            "local_path",
            10,
        )

        # Control loop timer
        period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(period, self.control_loop)

        self.get_logger().info("局部路径规划器初始化完成 (Pure Pursuit + 动态避障)")

    # ---------- Callbacks ----------

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
        lx, ly, lz = lookahead_pt

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
