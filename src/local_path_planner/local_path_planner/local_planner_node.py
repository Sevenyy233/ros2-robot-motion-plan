import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__("local_path_planner")

        # Parameters
        self.declare_parameter("layer_name", "elevation")
        self.declare_parameter("max_elevation", 1.0)
        self.declare_parameter("max_slope_angle", 45)
        self.declare_parameter("use_tf_for_start", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")

        self.layer_name = self.get_parameter("layer_name").value
        self.max_elevation = self.get_parameter("max_elevation").value
        self.max_slope_angle = self.get_parameter("max_slope_angle").value
        self.use_tf_for_start = self.get_parameter("use_tf_for_start").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value

        # Sate
        self.grid_map = None
        self.map_data = None
        self.map_info = None
        self.start_pose = None
        self.goal_pose = None
        self.global_path = None

        # 初始位置获取方式，从TF或/initialpose话题获取
        if self.use_tf_for_start:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.timer = self.create_timer(0.1, self.tf_start_pose_callback)
        else:
            self.sub_initialpose = self.create_subscription(
                PoseWithCovarianceStamped,
                "/initialpose",
                self.initialpose_callback,
                10
            )

        # 订阅者和发布者
        self.sub_global_path = self.create_subscription(
            Path,
            "/global_path",
            self.global_path_callback,
            10
        )

        