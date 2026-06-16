# OBB.msg
std_msgs/Header header   # 时间戳和坐标系
uint32 id                # 障碍物跟踪ID，保持稳定
geometry_msgs/Pose pose  # 障碍物中心点 + 姿态 (朝向)
geometry_msgs/Vector3 scale # 三个轴方向的全尺寸
