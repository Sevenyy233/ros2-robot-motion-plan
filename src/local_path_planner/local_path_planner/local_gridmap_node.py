import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np
import tf2_ros
import math

class LocalGridMapNode(Node):
    """
    订阅全局 /grid_map，根据机器人的当前位置截取局部区域 (Submap)，
    并基于 elevation 层计算 slope 层和 cost 层，最终发布 /local_grid_map。
    """
    def __init__(self):
        super().__init__('local_gridmap_node')

        # 参数配置
        self.declare_parameter('global_map_topic', '/grid_map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('local_map_size_x', 10.0) # 局部地图长度
        self.declare_parameter('local_map_size_y', 10.0) # 局部地图宽度
        self.declare_parameter('publish_rate', 10.0)     # 局部地图发布频率
        
        # Cost 层计算参数
        self.declare_parameter('max_slope_angle', 25.0)     # 超过此坡度代价值拉满
        self.declare_parameter('obstacle_height_thresh', 0.2) # 相对局部地面的障碍物高度阈值
        self.declare_parameter('lethal_cost', 255.0)        # 致命代价值

        self.global_map_topic = self.get_parameter('global_map_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.local_size_x = self.get_parameter('local_map_size_x').value
        self.local_size_y = self.get_parameter('local_map_size_y').value
        publish_rate = self.get_parameter('publish_rate').value
        
        self.max_slope = self.get_parameter('max_slope_angle').value
        self.obs_height = self.get_parameter('obstacle_height_thresh').value
        self.lethal_cost = self.get_parameter('lethal_cost').value

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 全局地图数据
        self.global_grid_data = None
        self.global_grid_info = None

        # 订阅与发布
        self.sub_global_map = self.create_subscription(
            GridMap, self.global_map_topic, self.global_map_callback, 1)
        self.pub_local_map = self.create_publisher(GridMap, '/local_grid_map', 1)

        # 定时器：高频发布局部地图
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_local_map)

        self.get_logger().info("Local GridMap (Submap截取 + Slope + Cost计算) 节点启动")

    def global_map_callback(self, msg):
        try:
            layer_idx = msg.layers.index("elevation")
        except ValueError:
            return
            
        self.global_grid_info = msg.info
        multi_array = msg.data[layer_idx]
        
        # 按照 GridMap 列主序解析
        size_x = int(round(msg.info.length_x / msg.info.resolution))
        size_y = int(round(msg.info.length_y / msg.info.resolution))
        for dim in multi_array.layout.dim:
            if dim.label == "column_index": size_y = dim.size
            elif dim.label == "row_index": size_x = dim.size
            
        if len(multi_array.data) == size_x * size_y:
            self.global_grid_data = np.array(multi_array.data, dtype=np.float32).reshape((size_x, size_y), order="F")

    def publish_local_map(self):
        if self.global_grid_data is None or self.global_grid_info is None:
            return

        # 获取机器人在 map 坐标系下的当前位置
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return

        cx = t.transform.translation.x
        cy = t.transform.translation.y
        res = self.global_grid_info.resolution

        # 确定局部地图在全局地图中的像素范围
        half_cells_x = int((self.local_size_x / 2.0) / res)
        half_cells_y = int((self.local_size_y / 2.0) / res)
        
        # 全局地图的左上角 (max_x, max_y)
        max_map_x = self.global_grid_info.pose.position.x + self.global_grid_info.length_x / 2.0
        max_map_y = self.global_grid_info.pose.position.y + self.global_grid_info.length_y / 2.0

        # 机器人中心在全局数组中的索引
        center_ix = int((max_map_x - cx) / res)
        center_iy = int((max_map_y - cy) / res)

        start_ix = center_ix - half_cells_x
        end_ix = center_ix + half_cells_x
        start_iy = center_iy - half_cells_y
        end_iy = center_iy + half_cells_y

        # 处理边界越界
        g_shape_x, g_shape_y = self.global_grid_data.shape
        valid_start_ix = max(0, start_ix)
        valid_end_ix = min(g_shape_x, end_ix)
        valid_start_iy = max(0, start_iy)
        valid_end_iy = min(g_shape_y, end_iy)

        # 提取局部 elevation
        local_shape_x = half_cells_x * 2
        local_shape_y = half_cells_y * 2
        local_elevation = np.full((local_shape_x, local_shape_y), np.nan, dtype=np.float32)

        if valid_start_ix < valid_end_ix and valid_start_iy < valid_end_iy:
            l_start_ix = valid_start_ix - start_ix
            l_end_ix = l_start_ix + (valid_end_ix - valid_start_ix)
            l_start_iy = valid_start_iy - start_iy
            l_end_iy = l_start_iy + (valid_end_iy - valid_start_iy)
            
            local_elevation[l_start_ix:l_end_ix, l_start_iy:l_end_iy] = \
                self.global_grid_data[valid_start_ix:valid_end_ix, valid_start_iy:valid_end_iy]

        # 计算 Slope 层 (通过梯度)
        dy, dx = np.gradient(local_elevation, res, res)
        # 坡度角 (度) = atan(sqrt(dx^2 + dy^2))
        local_slope = np.degrees(np.arctan(np.hypot(dx, dy)))

        # 计算 Cost 层
        local_cost = np.zeros_like(local_elevation)
        
        # 1. 坡度惩罚
        # 坡度越接近 max_slope，代价越大；超过则设为 lethal_cost
        with np.errstate(invalid='ignore'):
            slope_cost = (local_slope / self.max_slope) * 100.0
            local_cost = np.where(local_slope > self.max_slope, self.lethal_cost, slope_cost)

        # 2. 障碍物高度惩罚 (使用 Numpy 替代 scipy)
        # 通过简单的膨胀或均值滤波估算"地面高度"。这里为了避免依赖 scipy，
        # 我们使用 numpy 原生的简单 3x3 均值近似法 (利用 np.roll 进行平移计算)
        local_ground = np.copy(local_elevation)
        
        # 为了处理 NaN，我们将 NaN 替换为 0 进行计算，最后再恢复
        safe_elevation = np.nan_to_num(local_elevation)
        sum_elev = np.zeros_like(local_elevation)
        count_valid = np.zeros_like(local_elevation)
        
        # 3x3 窗口遍历
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                shifted = np.roll(np.roll(safe_elevation, dx, axis=0), dy, axis=1)
                valid_mask = np.roll(np.roll(~np.isnan(local_elevation), dx, axis=0), dy, axis=1)
                
                sum_elev += np.where(valid_mask, shifted, 0)
                count_valid += np.where(valid_mask, 1, 0)
                
        # 计算 3x3 均值作为地面高度基准
        with np.errstate(invalid='ignore', divide='ignore'):
            local_ground = sum_elev / count_valid
            
        height_diff = local_elevation - local_ground
        
        with np.errstate(invalid='ignore'):
            local_cost = np.where(height_diff > self.obs_height, self.lethal_cost, local_cost)
            
        # 3. 处理 NaN (未知区域设为中等偏高代价，谨慎驶入)
        local_cost[np.isnan(local_cost)] = 150.0

        # 打包发布
        msg = GridMap()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 注意：这里的地图是以机器人为中心提取的，但它的坐标系是相对于 map 的一个平移块
        # 为了与 base_link 对齐，我们将它的 frame 设为 base_link，但在填充数据时它已经对齐了
        msg.header.frame_id = self.base_frame 
        msg.info.resolution = res
        msg.info.length_x = self.local_size_x
        msg.info.length_y = self.local_size_y
        
        # 因为我们是将全局地图中以机器人为中心的那块抠出来，
        # 并声明 frame 为 base_link，所以这块局部地图的中心就是 (0,0)
        msg.info.pose.position.x = 0.0
        msg.info.pose.position.y = 0.0
        msg.info.pose.position.z = 0.0
        msg.info.pose.orientation.w = 1.0

        # 写入三个层
        layers = {"elevation": local_elevation, "slope": local_slope, "cost": local_cost}
        
        for name, data_arr in layers.items():
            msg.layers.append(name)
            arr = Float32MultiArray()
            # 替换 NaN 为 0.0 用于安全传输 (或者保留 nan，下游处理)
            arr.data = np.nan_to_num(data_arr.flatten(order='F')).tolist()
            dim_row = MultiArrayDimension(label="column_index", size=local_shape_y, stride=local_shape_x * local_shape_y)
            dim_col = MultiArrayDimension(label="row_index", size=local_shape_x, stride=local_shape_x)
            arr.layout.dim = [dim_row, dim_col]
            msg.data.append(arr)

        self.pub_local_map.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocalGridMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
