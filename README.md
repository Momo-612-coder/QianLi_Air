# __自主无人机竞速基本开发教程__
## 1. 启动模拟器 
参考 ***https://github.com/RoboMaster/IntelligentUAVChampionshipSimulator/tree/RMUA2026-01*** 配置好模拟器并启动

## 2. 安装Nvidia-Docker
>确保已安装了 Nvidia 驱动
----
>安装docker
>+ `sudo apt-get install ca-certificates gnupg lsb-release`
>+ `sudo mkdir -p /etc/apt/keyrings`
>+ `curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg`
>+ `echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null`
>+ `sudo apt-get update`
>+ `sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin`
----
>安装nvidia-container-toolkit
>+ `distribution=$(. /etc/os-release;echo $ID$VERSION_ID)`
>+ `curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -`
>+ `curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list`
>+ `sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit`
>+ `sudo systemctl restart docker`
---
>设置用户组，消除 *sudo* 限制  
>+ `sudo groupadd docker`  
>+ `sudo gpasswd -a $USER docker`  
>+ 注销账户并重新登录使新的用户组生效
>+ sudo service docker restart

## 3. 安装ROS-Noetic 
>+ `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`   
>+ `sudo apt install curl `  
>+ `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`   
>+ `sudo apt update`
>+ `sudo apt install ros-noetic-desktop-full`
>+ `sudo apt install python3-catkin-tools`

## 4. 基于docker镜像的控制程序开发流程
本次比赛中的模拟器使用 ***ROS*** 进行通讯，选手需要编写含有控制程序的ros功能包操控无人机完成目标,该ros功能包需要封装在docker镜像中进行提交。建议先在主机下开发完相应程序后在进行程序的docker封装，流程如下
>进入文件目录  
`cd /path/to/IntelligentUAVChampionshipBase/basic_dev`  
>开发案例完成功能设计与程序开发并根据需要修改 _Dockerfile_ 后，构建镜像   
`docker build -t basic_dev .`      
>导出镜像  
`docker image save [镜像：TAG] > test.tar`    
在主机工作目录下会出现 test.tar 文件，该文件即为可提交镜像  
### 注意:  
1. 服务器会在外部随机分配ip给容器，不能在镜像中的启动文件中提供 *ROS_IP* 和 *ROS_MASTER_URI* 这两个环境变量，否则服务器与容器将无法连接     
2. 镜像中的程序应在镜像启动后自动开启    
3. 镜像程序不允许使用GUI(X11等)功能        


## 5. 程序案例
### 基础开发环境(basic_dev)
#### 简介
该镜像包含有ros-noetic-desktop-focal以及相关的必要ros组件。程序中展示了如何模拟器进行数据交互
#### 使用说明
>进入文件目录    
`cd /path/to/IntelligentUAVChampionshipBase/basic_dev`  
>构建镜像   
`docker build -t basic_dev .`  
>启动docker镜像   
`./run_basic_dev.sh`  
>当看到如下图，说明容器启动成功，程序可接受到模拟器传出的数据
![pic](./docs/1.png)

#### 千里AIR扩展
>启动docker镜像，可以启动可视化界面，如rviz  
`./run_basic_dev_dev_gui.sh`  
>编译  
`catkin_make`  
>添加ros包  
`source ./devel/setup.bash`  
>rosrun启动  
`rosrun [功能包] [节点]`  
>roslaunch启动  
`roslaunch [功能包] [launch文件]`  

#### 新增功能包说明（千里AIR扩展）
以下功能包位于 `basic_dev/src/`，用于完成状态融合、控制与感知链路：

+ `imu_gps_fusion`
	+ 功能：融合 IMU 与 GPS，输出融合后的无人机状态，并发布轨迹与 TF。
	+ 输入：`/airsim_node/drone_1/imu/imu`、`/airsim_node/drone_1/gps`、`/airsim_node/drone_1/debug/pose_gt`、`/airsim_node/reset_cmd`
	+ 输出：`/airsim_node/drone_1/drone_state`、`true_path`、`fused_path`、`map -> base_link` TF
	+ 启动示例：`roslaunch imu_gps_fusion imu_gps_fusion.launch`（同时启动 `robot_state_publisher` 与 RViz）

+ `pwm_pid_controller`
	+ 功能：订阅融合状态后执行 PID 控制，发布四电机 PWM 控制量。
	+ 输入：`/airsim_node/drone_1/drone_state`
	+ 输出：`/airsim_node/drone_1/rotor_pwm_cmd`、`/airsim_node/reset_cmd`
	+ 启动示例：`rosrun pwm_pid_controller pwm_pid_controller`

+ `image_solver`
	+ 功能：对双目前视图像进行检测（ONNX 模型），估计得分门目标并发布可视化结果。
	+ 输入：`airsim_node/drone_1/front_left/Scene`、`airsim_node/drone_1/front_right/Scene`
	+ 输出：`image_solver/front_left/image`、`image_solver/front_right/image`、`image_solver/score_gate_target_marker`
	+ 启动示例：`rosrun image_solver image_solver`

+ `lidar_solver`
	+ 功能：对激光雷达点云进行滤波、分段与聚类，输出障碍物包围框可视化。
	+ 输入：`airsim_node/drone_1/lidar`
	+ 输出：`visualized_point_cloud`、`detected_bounding_boxes`
	+ 启动示例：`rosrun lidar_solver lidar_solver`

#### 新增脚本说明与建议使用方式
+ `run_basic_dev_dev_gui.sh`（开发/编译环境）
	+ 用途：启动带 GUI 的开发容器，适合在容器内编译、调试与手动运行节点。
	+ 常用流程：
		1. `cd /path/to/IntelligentUAVChampionshipBase/basic_dev`
		2. `./run_basic_dev_dev_gui.sh`
		3. `catkin_make`
		4. `source ./devel/setup.bash`
		5. 按需执行 `rosrun` 或 `roslaunch`

+ `start.sh`（一键启动全部节点）
	+ 用途：自动拉起后台容器，并在多个终端中一键启动完整功能链路。
	+ 当前脚本会启动：
		1. `roslaunch imu_gps_fusion imu_gps_fusion.launch`
		2. `rosrun pwm_pid_controller pwm_pid_controller`
		3. `rosrun image_solver image_solver`
		4. `rosrun lidar_solver lidar_solver`
	+ 使用方式：
		1. 先使用 `run_basic_dev_dev_gui.sh` 完成一次编译（确保 `devel/setup.bash` 已生成）
		2. 返回主机终端执行 `./start.sh`
		3. 在弹出的多个终端窗口中观察日志与运行状态

#### URDF与TF树说明
`imu_gps_fusion/launch/imu_gps_fusion.launch` 会加载 `urdf/drone.urdf.xacro`，并启动 `robot_state_publisher` 发布静态机体结构 TF。

>URDF核心结构（固定关节）
```text
base_link
├── front_left_camera_link
│   └── front_left_camera_optical_link
├── front_right_camera_link
│   └── front_right_camera_optical_link
├── back_left_camera_link
├── back_right_camera_link
└── lidar_link
```

>运行时TF关系
+ `imu_gps_fusion` 节点动态发布：`map -> base_link`（融合状态输出）
+ `imu_gps_fusion` 节点静态发布：`map -> map_rviz`（用于NED到RViz显示坐标系转换）
+ `robot_state_publisher` 根据 URDF 发布：`base_link -> 各传感器link`

>综合后常见TF树形态
```text
map
├── map_rviz
└── base_link
	├── front_left_camera_link
	│   └── front_left_camera_optical_link
	├── front_right_camera_link
	│   └── front_right_camera_optical_link
	├── back_left_camera_link
	├── back_right_camera_link
	└── lidar_link
```

>查看TF树（容器内）
`rosrun rqt_tf_tree rqt_tf_tree`

>或导出TF图
`rosrun tf view_frames`

#### 键盘控制示例（controller_test）
该示例位于 `basic_dev/src/controller/src/controllerTest.cpp`，通过键盘速度指令控制飞行并发布 PWM 控制。

>在容器内编译
`cd /home/catkin_ws`
`catkin_make`

>加载环境并启动节点
`source devel/setup.bash`
`rosrun controller_test controller_test`

>键盘操作（请保持终端焦点在该节点窗口）
+ 按住方向键 / `W A S D`：持续前后左右移动
+ 按住 `R` / `F`：持续上升 / 下降（若你的仿真坐标系相反，可在 `controllerTest.cpp` 互换二者）
+ 松开按键：自动悬停
+ 空格：立即悬停
+ `L`：切换日志显示/静默
+ `Q`：退出节点

>说明
+ 首次收到里程计后会自动将当前位置设为初始目标点。
+ 节点会持续发布 `/airsim_node/drone_1/rotor_pwm_cmd`。
+ 若在非交互终端运行（如后台启动），键盘控制会自动禁用。

## ros数据交互  
>用于获取数据的可订阅的主题  
>+ 前视相机   
`/airsim_node/drone_1/front_left/Scene`  
`/airsim_node/drone_1/front_right/Scene`
>+ 后视相机  
`/airsim_node/drone_1/back_left/Scene`  
`/airsim_node/drone_1/back_right/Scene`  
>+ imu数据  
`/airsim_node/drone_1/imu/imu`
>+ 雷达数据  
`/airsim_node/drone_1/lidar`
>+ 无人机状态真值  
`/airsim_node/drone_1/debug/pose_gt`  
>+ gps数据(含带误差姿态)  
`/airsim_node/drone_1/gps`  
>+ 电机输入PWM信号(0:右前, 1:左后, 2:左前, 3:右后)  
`/airsim_node/drone_1/debug/rotor_pwm`  
>+ 起始位姿  
`/airsim_node/initial_pose`  
>+ 终点位置  
`/airsim_node/end_goal`  
---- 
>用于发送指令的主题
>+ 速度控制(该主题通信需自行开发，开发所需文件可查阅模拟器中的VelCmdmsg文件夹)  
`/airsim_node/drone_1/vel_cmd_body_frame`
>+ PWM控制(0:右前, 1:左后, 2:左前, 3:右后)  
`/airsim_node/drone_1/rotor_pwm_cmd`
----
>可用服务   
>+ 起飞   
`/airsim_node/drone_1/takeoff`   
>+ 降落   
`/airsim_node/drone_1/land`   
>+ 重置   
`/airsim_node/reset` 
### 注意:   
1. 服务器仅开放规则手册中提及的话题,其余话题仅供调试程序使用。
2. 案例中不包含速度控制ros通信，需要自行开发，开发所需文件可查阅模拟器中的VelCmdmsg文件夹

## 系统相关参数
> 无人机系统参数  
>+ 质量 0.9kg    
>+ 轴距（电机至机体中心）0.18米  
>+ 转动惯量 Ixx 0.0046890742, Iyy 0.0069312, Izz 0.010421166  
>+ 电机升力系数 0.000367717  
>+ 电机反扭力系数 4.888486266072161e-06  
>+ 最大转速 11079.03 转每分钟  
----
> 标定板参数
>+ 行数（内点）8  
>+ 列数（内点）11  
>+ 方块边长 0.06 米  

## Point-LIO 与 Ego-Planner 补充说明

### 1. Point-LIO（激光惯导里程计）
`Point-LIO` 用于融合激光雷达与 IMU，输出实时位姿与地图点云，作为后续规划模块的定位输入。

>启动示例（容器内）
`roslaunch point_lio mapping_avia.launch`

>常见输入
+ 点云输入： `pointcloud2_to_livox`
+ IMU 输入：`/airsim_node/drone_1/imu/imu`

>常见输出
+ 里程计：`/Odometry`
+ 地图点云：`/cloud_registered`


### 2. Ego-Planner-Swarm
`Ego-Planner` 基于当前位姿与障碍物信息进行局部轨迹搜索与优化，输出可执行轨迹。

>启动示例（容器内）
`roslaunch ego_planner qianli_planner.launch`

>典型依赖
+ 里程计输入：通常来自 Point-LIO 输出（如 `/Odometry`）
+ 障碍物/地图输入：局部或全局点云地图

>使用建议
+ 建议在 Point-LIO 已稳定发布位姿后再启动 Ego-Planner。
+ 若规划器无轨迹输出，先检查目标点是否发布成功、以及地图坐标系是否与里程计坐标系一致。

### 3. 推荐启动顺序（开发调试）
1. `roslaunch pointcloud2_to_livox pointcloud2_to_livox.launch`
2. `roslaunch point_lio mapping_avia.launch`
3. `roslaunch ego_planner qianli_planner.launch`

### 4. 与一键脚本的关系
`basic_dev/start.sh` 已包含 `pointcloud2_to_livox`、`point_lio` 与 `ego_planner` 的启动逻辑。若你使用一键脚本，请先确保工作空间已完成编译并执行过 `source devel/setup.bash`。


