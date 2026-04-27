import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
import numpy as np

class TraversabilityNode(Node):
    def __init__(self):
        super().__init__('traversability_node')
        self.subscription = self.create_subscription(
            GridMap,
            '/grid_map',
            self.grid_map_callback,
            10)
        # 发布可以直接用于 Nav2 的全局代价地图
        self.publisher = self.create_publisher(OccupancyGrid, '/costmap', 10)
        
        # 定义通行阈值参数
        self.declare_parameter('max_slope_angle', 20.0) # 最大允许坡度(度)
        self.declare_parameter('max_elevation', 100.0)  # 放宽最大允许高度
        self.declare_parameter('min_elevation', -100.0) # 放宽最小允许高度，防止地势较低的平地被误认为障碍物
        
        self.get_logger().info("Traversability Analysis 节点已启动，正在监听 /grid_map")

    def grid_map_callback(self, msg: GridMap):
        if 'elevation' not in msg.layers:
            self.get_logger().warn("GridMap 中未找到 'elevation' 图层")
            return
        
        idx = msg.layers.index('elevation')
        array_msg = msg.data[idx]
        
        rows = array_msg.layout.dim[0].size
        cols = array_msg.layout.dim[1].size
        res = msg.info.resolution
        
        # 将一维数据转换为 2D Numpy 数组
        data = np.array(array_msg.data, dtype=np.float32).reshape((rows, cols))
        
        # 填充 nan 值（将缺失的点云数据补齐，避免梯度计算出现大面积 nan）
        data_filled = np.nan_to_num(data, nan=np.nanmean(data))
        
        # 1. 计算梯度 (计算每个网格的X和Y方向的差值)
        dy, dx = np.gradient(data_filled, res, res)
        # 2. 计算坡度 (弧度转角度)
        slope_rad = np.arctan(np.sqrt(dx**2 + dy**2))
        slope_deg = np.degrees(slope_rad)
        
        max_slope = self.get_parameter('max_slope_angle').value
        max_h = self.get_parameter('max_elevation').value
        min_h = self.get_parameter('min_elevation').value
        
        # 初始化代价数组 0: Free, 100: Lethal, -1: Unknown
        cost_data = np.zeros_like(data, dtype=np.int8)
        
        # 3. 找到不可通行的区域 (忽略 NaN 警告)
        with np.errstate(invalid='ignore'): 
            obstacle_mask = (slope_deg > max_slope) | (data > max_h) | (data < min_h)
        
        cost_data[obstacle_mask] = 100
        cost_data[np.isnan(data)] = -1  # 将原本没有点云数据的地方设为未知
        
        # 4. 坐标系对齐：
        # GridMap 原点在左上角(行主序)，OccupancyGrid 原点在左下角
        # 通常需要将矩阵上下翻转(flipud)以匹配 RViz 的二维地图显示标准
        cost_data = np.flipud(cost_data) 
        
        # 5. 构造 OccupancyGrid 消息
        grid = OccupancyGrid()
        grid.header = msg.header
        grid.header.frame_id = "map"
        grid.info.resolution = res
        grid.info.width = cols
        grid.info.height = rows
        
        # 计算 OccupancyGrid 的左下角真实世界坐标
        grid.info.origin.position.x = msg.info.pose.position.x - (cols * res) / 2.0
        grid.info.origin.position.y = msg.info.pose.position.y - (rows * res) / 2.0
        grid.info.origin.position.z = msg.info.pose.position.z
        grid.info.origin.orientation = msg.info.pose.orientation
        
        # 扁平化并转为 list
        grid.data = cost_data.flatten().tolist()
        
        self.publisher.publish(grid)

def main(args=None):
    rclpy.init(args=args)
    node = TraversabilityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()