# qianli_planner 参数说明

本文档说明 `qianli_planner.launch` 和 `qianli_param.xml` 中主要参数的作用、修改时机和耦合关系。当前配置面向大范围地图、点云输入和高速飞行场景：`max_vel=18.0`、`max_acc=8.0`、`planning_horizon=36.0`、`grid_map/resolution=0.8`。

## 文件关系

- `qianli_planner.launch`：场景入口。设置地图大小、话题、速度/加速度、规划前视距离和初始航点，然后 include `qianli_param.xml`。
- `qianli_param.xml`：真正给 `ego_planner_node` 写 ROS param，并完成话题 remap。
- `traj_server`：订阅规划器输出的 B-spline，发布 `/drone_0_planning/pos_cmd` 和 `/drone_0_planning/waypoint_cmd`。

## 启动级参数

| 参数 | 当前值 | 作用 | 什么时候改 |
| --- | --- | --- | --- |
| `map_size_x/y/z` | `780/720/150` | 栅格地图总体尺寸，单位 m。和 origin 一起决定地图覆盖范围。 | 飞行区域变大/变小，或出现 `pos out of map`、地图边缘截断时改。 |
| `map_origin_x/y/z` | `-20/-550/-5` | 地图最小角点坐标。地图覆盖范围是 `[origin, origin + size]`。 | 坐标系原点变化、赛道不在地图范围内、需要平移局部地图窗口时改。 |
| `odom_topic` | `odom_up` | 规划器和地图使用的里程计话题。 | 前端里程计换源时改，如 Point-LIO、AirSim、融合定位。 |
| `cloud_topic` | `/airsim_node/drone_1/lidar_odom` | 输入点云话题，用于占据地图更新。 | 传感器或仿真点云话题变化时改。注意不要重复加 `/`。 |
| `use_rviz` | `false` | 是否同时启动 RViz。 | 调试地图和轨迹时设为 `true`。 |
| `drone_id` | `0` | 无人机编号，影响节点名、规划话题和多机轨迹广播。 | 多机时每架机必须唯一，并从 0 开始更稳。 |

## 规划 FSM 参数

| 参数 | 当前值 | 作用 | 修改建议 |
| --- | --- | --- | --- |
| `fsm/flight_type` | `2` | 目标输入方式。`1` 为 RViz/manual goal，`2` 为预设航点/航点文件，`3` 在代码里声明但当前路径不完整。 | 比赛自动飞航点用 `2`；手动点目标调试用 `1`。 |
| `fsm/thresh_replan_time` | `0.3` | 执行当前局部轨迹超过该时间后允许重规划。越小越频繁。 | 高速/动态障碍多时可减小；CPU 压力大或轨迹抖动时增大。 |
| `fsm/thresh_no_replan_meter` | `1.5` | 离当前段终点小于该距离时，认为接近目标或中间停靠点。 | 航点密集或需要精准到点时减小；高速飞行可适当增大。 |
| `fsm/planning_horizon` | 来自 `planning_horizon`，当前 `36.0` | 局部目标前视距离。FSM 用它从全局轨迹上截取 local target，也用它过滤远处其他无人机轨迹。 | 随速度、制动距离、感知距离一起改。高速需要更大，狭窄环境可减小。 |
| `fsm/planning_horizen_time` | `3.0` | 保留参数，当前主要逻辑里基本不使用。 | 一般不改。 |
| `fsm/emergency_time` | `1.0` | 碰撞检查时，如果未来该时间窗口内碰撞，则进入 emergency stop。 | 速度更高或控制延迟更大时增大；误刹太多时小幅减小。 |
| `fsm/tracking_error_replan_thresh` | `3.0` | 期望用于轨迹跟踪误差触发重规划。 | 当前相关逻辑被注释，改了基本不生效。 |
| `fsm/tracking_error_replan_cooldown` | `0.5` | 跟踪误差重规划冷却时间。 | 当前相关逻辑被注释，改了基本不生效。 |
| `fsm/realworld_experiment` | `false` | 为 `false` 时不等待外部 trigger；为 `true` 时需要 `/traj_start_trigger`。 | 真实飞行且需要遥控/外部触发时设为 `true`。 |
| `fsm/fail_safe` | `true` | 是否启用安全检查/急停逻辑。 | 除非做离线调试，不建议关。 |

## 航点参数

| 参数 | 当前值 | 作用 | 修改建议 |
| --- | --- | --- | --- |
| `point_num`、`point0_x/y/z` ... | `1`，`10/0/1` | launch 内直接写入的预设航点。最多目前展开到 5 个点。 | 简单调试时直接改这些值。 |
| `use_waypoint_file` | `true` | 是否从文件读取航点。为 `true` 时会覆盖 launch 里 `point*` 的航点列表。 | 比赛路线建议用文件，便于维护。 |
| `waypoint_file` | `/basic_dev/waypoint.txt` | 航点文件路径。每行 `x y z` 或 `x,y,z`，空行和 `#` 注释会跳过。 | 航点文件位置改变时改。 |
| `waypoint_input_frame` | `map` | 航点所在坐标系。 | 航点来自全局地图时用 `map`；若已在规划坐标系下，可设成 `camera_init`。 |
| `planner_frame` | `camera_init` | 规划器内部使用的坐标系。航点会通过 TF 转到该 frame。 | 必须和里程计、点云 frame 体系一致。 |
| `waypoint_tf_timeout` | `0.1` | 等待航点 frame 转换的超时时间。 | TF 偶发延迟时可增大，如 `0.2`。 |
| `fsm/stop_yaw_position_tolerance` | `1.0` | 中间停靠点触发 yaw 旋转前，允许的位置误差。 | 希望停稳再转向时减小；高速下误判等待太久可增大。 |
| `fsm/stop_yaw_velocity_tolerance` | `0.25` | 中间停靠点触发 yaw 旋转前，允许的速度。 | 希望更稳再转向时减小；控制器刹停慢时增大。 |

注意：代码还读取 `fsm/stop_waypoint_index` 和 `fsm/stop_waypoint_wait_time`，但 `qianli_param.xml` 里没有写。更重要的是，`ego_replan_fsm.cpp` 当前会把 `stop_waypoint_wait_time_` 强制赋值为 `5.0`，所以即使通过参数传入等待时间，也会被覆盖。

## 地图参数

| 参数 | 当前值 | 作用 | 修改建议 |
| --- | --- | --- | --- |
| `grid_map/resolution` | `0.8` | 占据栅格分辨率。越小越精细，但内存和计算量快速增加。 | 大场景高速飞行用较大值；狭窄障碍、低速精细避障用较小值。 |
| `grid_map/map_size_x/y/z` | 来自 launch | 地图尺寸。 | 必须覆盖所有可能飞行区域。 |
| `grid_map/map_origin_x/y/z` | 来自 launch | 地图最小角点。 | 和 `map_size` 一起覆盖目标区域。 |
| `grid_map/local_update_range_x/y/z` | `24/24/16` | 每次根据当前位置更新/清理的局部范围。 | 应覆盖传感器有效距离和规划局部范围；太大耗时，太小会漏障碍。 |
| `grid_map/obstacles_inflation` | `0.0` | 障碍膨胀半径，影响 `occupancy_inflate`。 | 真实机体半径、定位误差、控制误差越大，应越大。当前为 0，安全裕度主要依赖优化距离。 |
| `grid_map/local_map_margin` | `10` | 局部地图索引边界额外 margin。 | 一般不动；边界附近频繁裁切可增大。 |
| `grid_map/ground_height` | `0.0` | 地面高度，用于初始化地图 z 下界和虚拟地面逻辑。 | 场地高度基准变化时改。 |
| `grid_map/min_ray_length` | `0.5` | raycast 融合忽略太近的点。 | 近处噪声多时增大；近障碍漏检时减小。 |
| `grid_map/max_ray_length` | `24.0` | raycast 最大更新距离。 | 应接近有效点云感知距离，并和 `local_update_range`、`planning_horizon` 协调。 |
| `grid_map/virtual_ceil_height` | `200.0` | 虚拟天花板高度，限制 z 方向可飞区域。代码会裁到 `ground_height + map_size_z`。 | 需要限制飞行高度时设为实际上限。 |
| `grid_map/visualization_truncate_height` | `200.0` | RViz 显示占据点时的截断高度。 | 只影响可视化，可按调试需要改。 |
| `grid_map/show_occ_time` | `false` | 是否显示占据时间相关信息。 | 性能调试时可开。 |
| `grid_map/pose_type` | `1` | 深度图同步位姿类型。`1` 为 odometry，`0` 为 pose stamped。当前仍同时订阅独立点云和 odom。 | 使用深度图融合时要和输入类型一致；当前点云模式一般不动。 |
| `grid_map/frame_id` | `camera_init` | 发布地图点云使用的 frame。 | 必须和规划坐标系/里程计 frame 对齐。 |

### 深度图相关参数

`use_depth_filter=false` 时，深度图滤波相关参数基本不参与当前点云主路径；如果改成深度图输入，则需要补齐/确认相机内参和深度图话题 remap。

| 参数 | 作用 |
| --- | --- |
| `grid_map/fx/fy/cx/cy` | 相机内参。`qianli_param.xml` 未设置，因为当前主要使用点云。 |
| `grid_map/use_depth_filter` | 是否过滤深度图边缘/无效点。 |
| `grid_map/depth_filter_tolerance` | 深度连续性过滤阈值。 |
| `grid_map/depth_filter_maxdist/mindist` | 深度有效距离范围。 |
| `grid_map/depth_filter_margin` | 深度图边缘忽略像素边界。 |
| `grid_map/k_depth_scaling_factor` | 深度图单位缩放，常见 uint16 毫米深度为 `1000.0`。 |
| `grid_map/skip_pixel` | 深度图采样步长，越大越快但更稀疏。 |

### 占据概率参数

| 参数 | 当前值 | 作用 | 修改建议 |
| --- | --- | --- | --- |
| `grid_map/p_hit` | `0.65` | ray 命中障碍时增加占据概率。 | 点云可靠、希望快速确认障碍时增大。 |
| `grid_map/p_miss` | `0.35` | ray 穿过自由空间时降低占据概率。 | 点云稀疏/动态噪声多时谨慎调。 |
| `grid_map/p_min` | `0.12` | 占据概率下限。 | 一般不动。 |
| `grid_map/p_max` | `0.90` | 占据概率上限。 | 障碍记忆太强可降低；需要稳定保留障碍可提高。 |
| `grid_map/p_occ` | `0.80` | 判定为占据的阈值。 | 漏障碍时降低；假障碍太多时提高。 |

## 轨迹管理与动力学参数

| 参数 | 当前值 | 作用 | 修改建议 |
| --- | --- | --- | --- |
| `manager/max_vel` | `18.0` | 规划最大速度。影响全局轨迹时间分配、局部目标速度、B-spline 可行性检查。 | 根据控制器能力和场地安全距离改。 |
| `manager/max_acc` | `8.0` | 规划最大加速度。影响制动速度、时间分配、可行性检查。 | 和实机最大可跟踪加速度一致，建议留裕度。 |
| `manager/max_jerk` | `50` | jerk 限制参数，当前主要作为配置保存，实际约束不如速度/加速度直接。 | 一般不优先调。 |
| `manager/control_points_distance` | `0.8` | 初始轨迹控制点间距，影响 B-spline 时间间隔和优化自由度。 | 障碍密集/需要细腻绕障时减小；高速大场景或 CPU 紧张时增大。 |
| `manager/feasibility_tolerance` | `0.05` | B-spline 可行性检查容忍比例。 | 轨迹经常因轻微超限被拉长可小幅增大；要严格动力学约束则减小。 |
| `manager/planning_horizon` | `36.0` | planner manager 内部规划前视距离。 | 应和 `fsm/planning_horizon` 保持一致。 |
| `manager/use_distinctive_trajs` | `true` | 遇到障碍段时生成多条候选拓扑轨迹再选优。 | 复杂障碍建议开；CPU 压力大、环境简单可关。 |

## 优化器参数

| 参数 | 当前值 | 作用 | 调参方向 |
| --- | --- | --- | --- |
| `optimization/lambda_smooth` | `5.0` | 平滑项权重。 | 轨迹抖动/弯折多时增大；绕障不积极时减小。 |
| `optimization/lambda_collision` | `0.5` | 障碍避让权重。 | 贴障或撞障时增大；绕得太保守时减小。 |
| `optimization/lambda_feasibility` | `0.1` | 速度/加速度可行性代价权重。 | 超速/超加速度多时增大；轨迹太慢太保守时减小。 |
| `optimization/lambda_fitness` | `1.0` | 贴合初始轨迹/目标形状的权重。 | 优化后偏离全局路线太多时增大；需要更自由绕障时减小。 |
| `optimization/dist0` | `0.3` | 障碍安全距离代价阈值。控制点距离障碍小于该值时产生碰撞代价。 | 需要更大障碍净距时增大；窄通道过不去时减小。 |
| `optimization/swarm_clearance` | `0.5` | 多机互相避让的安全距离。 | 多机间距不足时增大；队形很密时谨慎减小。 |
| `optimization/max_vel/max_acc` | `18/8` | 优化器内部动力学约束。 | 必须和 `manager/max_vel/max_acc` 同步。 |
| `optimization/order` | 默认 `3` | B-spline 阶数。`qianli_param.xml` 未显式设置，代码默认 3。 | 不建议改，多处代码假设三阶 B-spline。 |

## B-spline 参数

`qianli_param.xml` 设置了：

- `bspline/limit_vel = max_vel`
- `bspline/limit_acc = max_acc`
- `bspline/limit_ratio = 1.1`

但当前代码路径中没有读取这些 ROS 参数；`UniformBspline` 的实际可行性限制来自 `manager/max_vel`、`manager/max_acc` 和 `manager/feasibility_tolerance`。如果后续代码改成读取 `bspline/*`，再把这里纳入调参主路径。

## 预测参数

| 参数 | 当前值 | 作用 | 修改建议 |
| --- | --- | --- | --- |
| `prediction/obj_num` | `0` | 动态物体数量。当前 `planner_manager.cpp` 中 `ObjPredictor` 初始化被注释，基本不生效。 | 当前不用动态预测时保持 0。 |
| `prediction/lambda` | `1.0` | 动态物体预测模型参数。 | 预测模块启用后再调。 |
| `prediction/predict_rate` | `1.0` | 预测频率。 | 预测模块启用后再调。 |

## traj_server 参数

| 参数 | 当前值 | 作用 | 修改建议 |
| --- | --- | --- | --- |
| `traj_server/time_forward` | `1.0` | 轨迹采样前瞻时间，发布给控制器的期望点会向前取一点。 | 控制滞后明显时可增大；跟踪超前、切弯明显时减小。 |
| `traj_server/yaw_rotate_duration` | `2.0` | 收到 `yaw_turn_180` 后的旋转持续时间。 | 中间停靠转向太急时增大，太慢时减小。 |

## 关键耦合关系

### 速度、加速度、前视距离

`max_vel`、`max_acc`、`planning_horizon` 必须一起调。经验上，前视距离至少要覆盖制动距离并留规划余量：

```text
brake_distance = max_vel^2 / (2 * max_acc)
planning_horizon ~= 1.5 ~ 2.0 * brake_distance
```

当前 `18 m/s`、`8 m/s^2` 的制动距离约 `20.25 m`，`planning_horizon=36 m` 约为 `1.78x`，是合理的高速配置。

### 前视距离、感知距离、地图更新范围

`planning_horizon` 不应明显超过可靠感知/建图距离，否则局部目标会落到未充分观测区域。当前点云 raycast 最大距离 `max_ray_length=24 m`，局部更新范围 `24 m`，而 `planning_horizon=36 m`。这适合已有全局航点轨迹、局部只需避开近处障碍的用法；如果环境未知障碍很多，建议提高感知/更新范围，或降低 `planning_horizon`。

相关参数：

- `fsm/planning_horizon`
- `manager/planning_horizon`
- `grid_map/max_ray_length`
- `grid_map/local_update_range_x/y/z`
- 传感器真实有效距离

### 地图尺寸、origin、resolution、内存

栅格数量约为：

```text
ceil(map_size_x / resolution) *
ceil(map_size_y / resolution) *
ceil(map_size_z / resolution)
```

当前 `780 x 720 x 150`、`resolution=0.8` 约为 `975 * 900 * 188 = 164,970,000` 个 voxel，内存压力很高。代码已经加了总 voxel 数超过 `int` 上限的保护，但即使没超限也可能消耗大量内存。

如果启动慢、内存高或崩溃，优先：

1. 增大 `grid_map/resolution`。
2. 缩小 `map_size_z` 或平移 `map_origin`，只覆盖实际高度/区域。
3. 缩小 `map_size_x/y`，分段跑图。

### 地图 frame、航点 frame、里程计 frame

这些必须在同一个 TF 体系下：

- `grid_map/frame_id`
- `planner_frame`
- `waypoint_input_frame`
- `odom_topic` 消息 header frame / child frame
- `cloud_topic` 点云 frame

若航点全部被 drop、地图和轨迹在 RViz 中错位，优先检查 TF：`map -> camera_init` 是否存在、点云是否在 `camera_init` 下正确显示。

### 控制点间距、地图分辨率、速度

`manager/control_points_distance` 和 `grid_map/resolution` 最好处在同一量级。当前二者都是 `0.8`，比较一致。

- 控制点间距太大：轨迹不够灵活，窄障碍绕不过。
- 控制点间距太小：优化变量变多，CPU 压力增大，轨迹可能更抖。
- 分辨率太粗：障碍边界粗糙，窄通道信息丢失。
- 分辨率太细：大地图内存和 raycast 开销显著增加。

### 安全距离和障碍膨胀

实际避障安全裕度来自两层：

- `grid_map/obstacles_inflation`：地图层膨胀。
- `optimization/dist0`：优化器碰撞代价距离。

当前 `obstacles_inflation=0.0`、`dist0=0.3`，属于比较激进的高速配置。如果实机半径、定位误差或控制误差更大，建议优先增大 `obstacles_inflation` 到机体半径附近，再配合增大 `dist0`。

### 重规划频率和计算负载

重规划频率主要受这些影响：

- `fsm/thresh_replan_time` 越小，重规划越频繁。
- `manager/use_distinctive_trajs=true` 会增加候选轨迹搜索和优化开销。
- `grid_map/resolution` 越小、`local_update_range` 越大，建图越重。
- `control_points_distance` 越小，优化变量越多。

如果 CPU 爆、规划延迟大，优先增大 `thresh_replan_time`、关闭 `use_distinctive_trajs` 做对照、增大 `resolution` 或 `control_points_distance`。

## 常见调参场景

### 想飞得更快

同时调整：

- 增大 `max_vel`。
- 按制动距离公式增大 `planning_horizon`。
- 确认 `max_acc` 是控制器能稳定跟踪的值。
- 增大 `grid_map/max_ray_length` 和 `local_update_range`，或确认远处障碍已有全局先验。
- 适当增大 `emergency_time`。

### 经常贴障或碰障

优先检查点云和 TF 是否正确；确认无误后：

- 增大 `grid_map/obstacles_inflation`。
- 增大 `optimization/dist0`。
- 增大 `optimization/lambda_collision`。
- 降低 `max_vel` 或增大 `planning_horizon`。

### 轨迹太保守、绕太远

- 减小 `optimization/lambda_collision` 或 `optimization/dist0`。
- 减小 `grid_map/obstacles_inflation`。
- 增大 `lambda_fitness`，让优化更贴近初始/全局轨迹。
- 确认 `p_occ` 不要太低，否则噪声会变成障碍。

### 轨迹抖动或控制跟踪差

- 增大 `optimization/lambda_smooth`。
- 降低 `max_vel`、`max_acc`。
- 增大 `manager/control_points_distance`。
- 减小 `traj_server/time_forward`，如果表现为明显超前切弯。

### 航点不生效或起飞后不规划

- `flight_type=2` 时，节点会等待 odom、`control_start`，并在收到后延迟 2 秒再读航点。
- 如果 `use_waypoint_file=true`，检查 `/basic_dev/waypoint.txt` 是否存在且格式正确。
- 检查 `waypoint_input_frame -> planner_frame` 的 TF。
- 若只是手动 RViz 点目标，改成 `flight_type=1`。

### RViz 中地图/轨迹错位

- 检查 `grid_map/frame_id`、`planner_frame`、里程计 frame 是否一致。
- 检查点云输入 frame 是否能正确转换到 `camera_init`。
- 检查 `map_origin` 和 `map_size` 是否覆盖当前 odom 坐标。

## 推荐修改顺序

1. 先定坐标系和话题：`odom_topic`、`cloud_topic`、`planner_frame`、`grid_map/frame_id`。
2. 再定地图范围：`map_origin`、`map_size`、`resolution`。
3. 再定动力学：`max_vel`、`max_acc`。
4. 根据制动距离定 `planning_horizon`。
5. 根据传感器距离定 `max_ray_length`、`local_update_range`。
6. 最后微调优化权重：`lambda_*`、`dist0`、`obstacles_inflation`。

优先一次只改一组耦合参数，并保存能稳定飞行的配置作为基线。
