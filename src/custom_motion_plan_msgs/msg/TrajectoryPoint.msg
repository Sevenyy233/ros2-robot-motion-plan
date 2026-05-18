# 期望位姿(位置+姿态)
geometry_msgs/Pose pose

# 期望速度 (包含线速度和角速度)
geometry_msgs/Twist velocity

# 期望加速度 (包含线加速度和角加速度)
geometry_msgs/Accel acceleration

# 从轨迹起点计算的相对时间
builtin_interfaces/Duration time_from_start