
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Header
import numpy as np
import tf2_ros

# 根据地图发布真实的3D障碍物点云信息作为局部路径规划的输入
# 按照 xacro 中雷达的真实 FOV 参数进行过滤筛选
class FakeLidarPointsPub(Node):
    def __init__(self):
        super().__init__("fake_lidar_points_pub")

        self.declare_parameter("lidar_frame", "lidar_3_link")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("max_elevation", 1.0)
        self.declare_parameter("layer_name", "elevation")

        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 30.0)
        self.declare_parameter("horizontal_fov_deg", 180.0)
        self.declare_parameter("vertical_fov_deg", 30.0)

        self.declare_parameter("cube_enabled", True)
        self.declare_parameter("cube_x", 2.0)
        self.declare_parameter("cube_y", 2.0)
        self.declare_parameter("cube_z", 0.3)
        self.declare_parameter("cube_size", 0.5)
        self.declare_parameter("cube_resolution", 0.05)

        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.max_elevation = self.get_parameter("max_elevation").value
        self.layer_name = self.get_parameter("layer_name").value

        self.range_min = self.get_parameter("range_min").value
        self.range_max = self.get_parameter("range_max").value
        self.horizontal_half_fov = np.deg2rad(self.get_parameter("horizontal_fov_deg").value / 2.0)
        self.vertical_half_fov = np.deg2rad(self.get_parameter("vertical_fov_deg").value / 2.0)

        self.cube_enabled = self.get_parameter("cube_enabled").value
        self.cube_x = self.get_parameter("cube_x").value
        self.cube_y = self.get_parameter("cube_y").value
        self.cube_z = self.get_parameter("cube_z").value
        self.cube_size = self.get_parameter("cube_size").value
        self.cube_resolution = self.get_parameter("cube_resolution").value

        self.points_pub_ = self.create_publisher(PointCloud2, "/lidar3_points", 10)
        self.cube_pub_ = self.create_publisher(PointCloud2, "/obstacle_cube/points", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.map_sub = self.create_subscription(GridMap, "/grid_map", self.grid_map_callback, 1)

        self.obstacle_points_map = None
        self.cube_points_map = self.generate_cube_points()

        self.timer_ = self.create_timer(0.1, self.points_timer_callback)

        self.get_logger().info("订阅地图并计算发布3D障碍物点云信息（含FOV过滤），初始化成功！")

    def generate_cube_points(self):
        half = self.cube_size / 2.0
        res = self.cube_resolution
        points = []

        if not self.cube_enabled:
            return np.empty((0, 3))

        steps = max(1, int(self.cube_size / res))
        xs = np.linspace(-half, half, steps + 1)
        ys = np.linspace(-half, half, steps + 1)
        zs = np.linspace(-half, half, steps + 1)

        for x in xs:
            for y in ys:
                points.append([x, y, -half])
                points.append([x, y, half])

        for x in xs:
            for z in zs[1:-1]:
                points.append([x, -half, z])
                points.append([x, half, z])

        for y in ys[1:-1]:
            for z in zs[1:-1]:
                points.append([-half, y, z])
                points.append([half, y, z])

        arr = np.array(points) + np.array([self.cube_x, self.cube_y, self.cube_z])
        return arr

    def grid_map_callback(self, msg):
        try:
            layer_idx = msg.layers.index(self.layer_name)
        except ValueError:
            self.get_logger().warn(f"GridMap中未找到层 '{self.layer_name}'", throttle_duration_sec=5.0)
            return

        map_info = msg.info
        multi_array = msg.data[layer_idx]

        size_x = int(round(map_info.length_x / map_info.resolution))
        size_y = int(round(map_info.length_y / map_info.resolution))

        if len(multi_array.layout.dim) >= 2:
            for dim in multi_array.layout.dim:
                if dim.label == 'column_index':
                    size_y = dim.size
                elif dim.label == 'row_index':
                    size_x = dim.size

        if len(multi_array.data) != size_x * size_y:
            return

        map_data = np.array(multi_array.data, dtype=np.float32).reshape((size_x, size_y), order='F')

        res = map_info.resolution
        center_x = map_info.pose.position.x
        center_y = map_info.pose.position.y
        max_x = center_x + map_info.length_x / 2.0
        max_y = center_y + map_info.length_y / 2.0

        valid_mask = ~np.isnan(map_data) & (map_data > self.max_elevation)
        idx_x, idx_y = np.where(valid_mask)
        z = map_data[valid_mask]
        x = max_x - (idx_x + 0.5) * res
        y = max_y - (idx_y + 0.5) * res

        self.obstacle_points_map = np.column_stack((x, y, z))

    def quat_to_mat(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,         2*x*z + 2*y*w],
            [2*x*y + 2*z*w,         1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [2*x*z - 2*y*w,         2*y*z + 2*x*w,         1 - 2*x*x - 2*y*y]
        ])

    def points_timer_callback(self):
        map_obstacles = self.obstacle_points_map
        if map_obstacles is None:
            map_obstacles = np.empty((0, 3))

        cube_pts = self.cube_points_map
        if cube_pts is None or len(cube_pts) == 0:
            all_points_map = map_obstacles
        else:
            all_points_map = np.vstack((map_obstacles, cube_pts)) if len(map_obstacles) > 0 else cube_pts

        if self.cube_enabled and len(cube_pts) > 0:
            header_map = Header()
            header_map.stamp = self.get_clock().now().to_msg()
            header_map.frame_id = self.map_frame
            self.cube_pub_.publish(pc2.create_cloud_xyz32(header_map, cube_pts.tolist()))

        if len(all_points_map) == 0:
            return

        try:
            trans = self.tf_buffer.lookup_transform(self.lidar_frame, self.map_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"等待 TF 变换: {e}", throttle_duration_sec=2.0)
            return

        t = trans.transform.translation
        q = trans.transform.rotation

        T = np.array([t.x, t.y, t.z])
        R = self.quat_to_mat(q)

        points_lidar = (R @ all_points_map.T).T + T

        dist = np.sqrt(np.sum(points_lidar**2, axis=1))
        mask_range = (dist >= self.range_min) & (dist <= self.range_max)

        px, py, pz = points_lidar[:, 0], points_lidar[:, 1], points_lidar[:, 2]
        azimuth = np.abs(np.arctan2(py, px))
        mask_horizontal = azimuth <= self.horizontal_half_fov

        dist_xy = np.sqrt(px**2 + py**2)
        elevation = np.abs(np.arctan2(pz, dist_xy))
        mask_vertical = elevation <= self.vertical_half_fov

        mask = mask_range & mask_horizontal & mask_vertical
        valid_points = points_lidar[mask]

        if len(valid_points) == 0:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.lidar_frame

        points_msg = pc2.create_cloud_xyz32(header, valid_points.tolist())
        self.points_pub_.publish(points_msg)


def main():
    rclpy.init()
    node = FakeLidarPointsPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
