# 期望位姿(位置+姿态)
geometry_msgs/Point position            # 三维位置x,y,z
geometry_msgs/Quaternion orientation    # 四元数w,x,y,z,四元数表示姿态

# 期望速度
geometry_msgs/Vector3 linear_velocity   # 线速度
geometry_msgs/Vector3 angular_velocity  # 角速度

# 期望加速度
geometry_msgs/Vector3 linear_acceleration  # 线加速度
geometry_msgs/Vector3 angular_acceleration  # 角加速度

# 从轨迹起点计算的相对时间
builtin_interfaces/Duration time_from_start