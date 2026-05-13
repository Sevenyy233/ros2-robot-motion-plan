std_msgs/Header header

# 期望位姿(位置+姿态)
# 三维位置x,y,z
geometry_msgs/Point position
# 四元数w,x,y,z,四元数表示姿态
geometry_msgs/Quaternion orientation

# 期望速度
# 线速度
geometry_msgs/Vector3 linear_velocity
# 角速度
geometry_msgs/Vector3 angular_velocity

# 期望加速度
# 线加速度
geometry_msgs/Vector3 linear_acceleration
# 角加速度
geometry_msgs/Vector3 angular_acceleration

# 从轨迹起点计算的相对时间
builtin_interfaces/Duration time_from_start