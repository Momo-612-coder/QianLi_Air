# launch 参数调节说明

本文档说明两个主要 launch 文件里的参数如何调节：

- `src/lidar_solver/launch/lidar_solver.launch`：雷达点云过滤、聚类、障碍物框发布。
- `src/AB_planner/launch/AB_planner.launch`：A*、B-spline、动态避障、MPC 跟踪。

整体链路是：

```text
雷达点云
-> lidar_solver: FOV / ROI / voxel / ground segmentation / clustering / object filter
-> /front_fov_obstacles
-> AB_planner: 多帧障碍物缓存 / A* 重规划 / B-spline 碰撞检查 / MPC 跟踪
```

调参时建议先调 `lidar_solver`，确认障碍物框稳定、大小合理、没有大量误检，再调 `AB_planner`。感知输出不稳定时，规划参数调得再激进也容易表现为频繁重规划、路径抖动或原地等待。

## lidar_solver.launch

### 坐标与 FOV

| 参数 | 默认值 | 作用 | 调大效果 | 调小效果 |
| --- | ---: | --- | --- | --- |
| `target_frame` | `map` | 点云转换后的目标坐标系 | 通常不调 | 通常不调 |
| `source_frame_override` | `lidar_link` | 强制指定雷达原始坐标系 | 雷达 TF 名称不一致时修改 | 通常不调 |
| `front_fov_deg` | `120.0` | 最终发布前方障碍物时使用的前方视场角 | 发布更多侧前方障碍物 | 减少侧方误检 |
| `front_fov_margin_deg` | `8.0` | 发布障碍物时给 bbox 角点检查的额外余量 | 更保守，侧边障碍也容易发布 | 更严格，可能漏掉边缘障碍 |
| `fov_prefilter_enabled` | `true` | 是否先按 FOV 过滤点云 | 关闭后处理全点云，误聚类风险增加 | 开启更快、更干净 |
| `fov_prefilter_deg` | `140.0` | 点云预过滤 FOV | 保留更多点，减少边缘障碍被切掉 | 点更少，速度更快但可能漏边缘 |

联动建议：

- `fov_prefilter_deg` 应该大于 `front_fov_deg`，一般比 `front_fov_deg + 2 * front_fov_margin_deg` 略大一点。
- 如果边缘障碍物时有时无，先增大 `fov_prefilter_deg` 或 `front_fov_margin_deg`。
- 如果赛道侧墙、边界结构大量进入障碍物列表，优先减小 `front_fov_deg` 或收紧 ROI 的 `roi_min_y / roi_max_y`。

### ROI 与点云降采样

| 参数 | 默认值 | 作用 | 调大效果 | 调小效果 |
| --- | ---: | --- | --- | --- |
| `voxel_leaf_size` | `0.05` | voxel 降采样体素大小 | 点更少、速度更快、远处更容易断裂 | 点更多、聚类更细、计算更重 |
| `roi_filter_enabled` | `true` | 是否启用雷达坐标系 ROI 裁剪 | 关闭会保留更多无关结构 | 开启可减少墙面、地面、赛道结构干扰 |
| `roi_min_x` | `0.0` | 前向最小距离 | 过滤近处雷达噪声 | 保留近处障碍 |
| `roi_max_x` | `35.0` | 前向最大距离 | 看得更远，但远处稀疏误检增加 | 更稳、更快，但远处障碍发现晚 |
| `roi_min_y` | `-18.0` | 左右横向最小范围 | 保留更多侧方点 | 减少侧墙/赛道边界误检 |
| `roi_max_y` | `18.0` | 左右横向最大范围 | 保留更多侧方点 | 减少侧墙/赛道边界误检 |
| `roi_min_z` | `-6.0` | 垂直下界 | 保留更多下方点，地面干扰增加 | 过滤地面/低矮噪声 |
| `roi_max_z` | `6.0` | 垂直上界 | 保留更多上方点 | 过滤上方无关点 |

联动建议：

- 如果远处障碍物断裂，先不要盲目调大 `voxel_leaf_size`；可以适当增大远距离 `cluster_seg2/3_tolerance`，或降低远距离 `cluster_seg2/3_min_size`。
- 如果墙面或赛道边界被聚成大障碍，优先收紧 `roi_min_y / roi_max_y` 和 `roi_min_z / roi_max_z`，再调 cluster。
- `roi_max_x` 不宜远大于实际需要。规划只需要足够提前发现障碍，太远的稀疏点容易导致不稳定聚类。

### 地面平面剔除

| 参数 | 默认值 | 作用 | 调大效果 | 调小效果 |
| --- | ---: | --- | --- | --- |
| `ground_segmentation_enabled` | `false` | 是否启用 RANSAC 地面平面剔除 | 开启后可减少地面/赛道面误聚类 | 关闭更安全，不会误删障碍 |
| `ground_segmentation_max_iterations` | `30` | RANSAC 迭代次数 | 更容易找到平面，计算更重 | 更快但可能分割不稳 |
| `ground_segmentation_distance_threshold` | `0.15` | 点到平面的距离阈值 | 删除更厚的一层地面 | 删除更薄，保留更多低矮物体 |
| `ground_segmentation_min_inlier_ratio` | `0.08` | 平面点比例下限 | 更谨慎，不容易误删 | 更容易启用地面删除 |
| `ground_segmentation_min_z_normal` | `0.7` | 平面法向量 z 分量下限 | 只删更接近水平的面 | 可能把斜面也删掉 |

联动建议：

- 先保持 `ground_segmentation_enabled=false`，确认 ROI 和 cluster 无法解决地面误检时再打开。
- 开启后如果障碍物底部被削掉，减小 `ground_segmentation_distance_threshold`。
- 如果墙面被误删，增大 `ground_segmentation_min_z_normal`。
- 如果地面仍然被保留，适当降低 `ground_segmentation_min_inlier_ratio` 或增大 `distance_threshold`。

### 聚类参数

当前代码按距离分段聚类：

- `seg0`: 0 到 2 m
- `seg1`: 2 到 8 m
- `seg2`: 8 到 20 m
- `seg3`: 20 到 30 m
- `seg4`: 预留区间

每段都有三个参数：

| 参数 | 作用 | 调大效果 | 调小效果 |
| --- | --- | --- | --- |
| `cluster_segX_tolerance` | 聚类半径 | 更容易把稀疏点连成一个障碍，但也容易把墙/多个物体粘在一起 | 障碍分得更细，但远处容易断裂 |
| `cluster_segX_min_size` | 最小点数 | 过滤噪声更强，但远处小障碍可能漏检 | 远处稀疏障碍更容易保留，但噪声变多 |
| `cluster_segX_max_size` | 最大点数 | 允许大障碍存在 | 过滤墙面/地面大簇更强，但大障碍可能被丢弃 |

调参建议：

- 近处误检多：增大 `cluster_seg0_min_size`，减小 `cluster_seg0_tolerance`。
- 远处障碍断裂：增大 `cluster_seg2_tolerance / cluster_seg3_tolerance`，降低 `cluster_seg2_min_size / cluster_seg3_min_size`。
- 墙面被聚成大障碍：降低对应距离段的 `cluster_segX_max_size`，同时收紧 ROI。
- 多个近距离障碍被粘成一个：减小对应段的 `cluster_segX_tolerance`。

### 障碍物尺寸过滤与平滑

| 参数 | 默认值 | 作用 | 调大效果 | 调小效果 |
| --- | ---: | --- | --- | --- |
| `object_filter_min_extent` | `0.05` | xy 平面最小尺寸 | 过滤小噪声更多 | 保留更小障碍 |
| `object_filter_min_height` | `0.05` | 最小高度 | 过滤低矮地面点更多 | 保留低矮障碍 |
| `object_filter_max_length` | `12.0` | 最大 x 尺寸 | 允许更长障碍 | 更强过滤墙/赛道边界 |
| `object_filter_max_width` | `12.0` | 最大 y 尺寸 | 允许更宽障碍 | 更强过滤墙/赛道边界 |
| `object_filter_max_height` | `8.0` | 最大高度 | 允许更高障碍 | 过滤高大异常簇 |
| `object_filter_max_volume` | `200.0` | 最大体积 | 允许大障碍 | 过滤大面积误聚类 |
| `object_filter_reject_thin_vertical_band_enabled` | `true` | 是否过滤 y 很薄、z 很长的竖向薄板 | 开启可过滤赛道连接带 | 关闭后保留所有薄板状物体 |
| `object_filter_thin_band_max_y_thickness` | `0.45` | 薄板 y 方向最大厚度 | 更容易过滤稍厚连接带 | 只过滤非常薄的结构 |
| `object_filter_thin_band_min_z_extent` | `1.5` | 薄板 z 方向最小长度 | 只过滤更高/更长结构 | 更容易过滤较短薄板 |
| `object_filter_thin_band_min_z_to_y_ratio` | `4.0` | z/y 最小比例 | 只过滤更细长结构 | 更容易过滤竖向薄结构 |
| `object_filter_thin_band_max_min_to_max_ratio` | `0.18` | 最薄维度/最长维度比例上限 | 只过滤更薄的板 | 更容易过滤板状结构 |
| `temporal_smoothing_enabled` | `true` | 是否平滑 bbox | 框更稳定 | 关闭后响应快但抖 |
| `temporal_smoothing_alpha` | `0.65` | 当前帧权重 | 越接近 1 响应越快 | 越小越平滑但延迟更大 |
| `temporal_smoothing_max_match_distance` | `1.8` | 跨帧匹配距离 | 快速运动/抖动时更容易匹配 | 减少错误匹配 |

联动建议：

- 如果 bbox 抖动导致 AB_planner 频繁重规划，降低 `temporal_smoothing_alpha`，或增大 `temporal_smoothing_max_match_distance`。
- 如果障碍物移动后框拖影明显，增大 `temporal_smoothing_alpha`，或减小 AB_planner 的 `dynamic_avoidance_obstacle_history_time`。
- 如果墙面大簇进入规划，优先收紧 `object_filter_max_length / max_width / max_volume`。
- 如果赛道两侧连接带被识别为障碍，优先保持 `reject_thin_vertical_band_enabled=true`，再根据实际 bbox 尺寸调 `thin_band_max_y_thickness` 和 `thin_band_min_z_extent`。

## AB_planner.launch

`AB_planner.launch` 会 include `A_star_params.xml`，实际节点参数在 `A_star_params.xml` 中注入。日常调参改 `AB_planner.launch` 里的 arg 即可。

### A* 地图与搜索

| 参数 | 默认值 | 作用 | 调大效果 | 调小效果 |
| --- | ---: | --- | --- | --- |
| `plan_size_x` | `200.0` | 规划地图 x 范围 | 可规划更大区域，内存/搜索更重 | 范围小，可能找不到远目标 |
| `plan_size_y` | `200.0` | 规划地图 y 范围 | 同上 | 同上 |
| `plan_size_z` | `100.0` | 规划地图 z 范围 | 同上 | 同上 |
| `resolution` | `0.2` | A* 栅格分辨率 | 数值大：搜索快但路径粗 | 数值小：路径细但计算重 |
| `reserved_back_space` | `10.0` | 保留后方空间 | 更保守 | 后方裁剪更多 |
| `safety_distance` | `0.2` | 静态/动态避障基础安全距离 | 更安全但更容易无路 | 更贴近障碍 |
| `h_weight` | `1.35` | A* 启发式权重 | 搜索更快但路径可能不够优 | 更稳但更慢 |

联动建议：

- `resolution` 要和 `dynamic_avoidance_obstacle_inflation`、`bspline_collision_sample_step` 同量级考虑。分辨率粗时，安全距离和膨胀半径不能太小。
- 如果 A* 经常无路径，先减小 `safety_distance` 或 `dynamic_avoidance_obstacle_inflation`，再考虑调 `resolution`。
- 如果规划太慢，增大 `resolution` 或 `h_weight`。

### 话题与可视化

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| `odometry_topic` | `/airsim_node/drone_1/drone_state` | 无人机里程计/状态输入 |
| `fusion_start_topic` | `/control_start` | 开始控制/规划信号 |
| `obstacle_topic` | `/front_fov_obstacles` | 接收 lidar_solver 发布的障碍物 |
| `visualization_frame` | `map` | RViz 可视化坐标系 |
| `bspline_visualization_samples` | `500` | B-spline 可视化采样点数 |

联动建议：

- `obstacle_topic` 必须和 `lidar_solver` 发布的 `front_fov_obstacles` 对上。
- `visualization_frame` 应与 `lidar_solver target_frame` 一致，默认都用 `map`。

### 航点、重规划与航向

| 参数 | 默认值 | 作用 | 调大效果 | 调小效果 |
| --- | ---: | --- | --- | --- |
| `plan_waypoint_lookahead` | `2` | 一次向后看多少个航点进行规划 | 路径更长、更全局 | 更局部，重规划更频繁 |
| `replan_trigger_distance` | `20.0` | 距当前 B-spline 末端多近时触发下一段规划 | 更早规划下一段 | 更晚规划，可能临近末端才重规划 |
| `waypoint_reached_distance` | `8.0` | 判定航点到达的距离 | 更容易切下一个航点 | 更严格，可能卡在航点附近 |
| `use_waypoint_yaw` | `true` | 是否使用航点 yaw | 航向跟任务点 | 航向跟路径/控制 |
| `max_yaw_rate` | `3.5` | 最大 yaw 角速度 | 转向更快 | 转向更平滑 |

联动建议：

- `replan_trigger_distance` 应该大于 MPC 预测距离和刹停距离，否则到路径末端附近才发现要重规划。
- `waypoint_reached_distance` 太小会导致无人机在航点附近反复调整，太大会提前切段。

### 路径优化与 B-spline

| 参数 | 默认值 | 作用 | 调大效果 | 调小效果 |
| --- | ---: | --- | --- | --- |
| `bspline_control_point_spacing` | `2.0` | B-spline 控制点间距 | 曲线更平滑但可能切角 | 更贴近 A* 路径但更抖 |
| `path_optimization_enabled` | `true` | 是否优化 A* 路径点 | 路径更平滑 | 更贴近原始 A* |
| `path_optimization_iterations` | `80` | 优化迭代次数 | 更充分但更慢 | 更快但效果弱 |
| `path_optimization_data_weight` | `0.20` | 贴近原路径权重 | 更贴近 A*，不易切进障碍 | 更容易平滑偏离 |
| `path_optimization_smooth_weight` | `0.45` | xy 平滑权重 | 更平滑 | 更贴原始路径 |
| `path_optimization_z_smooth_weight` | `0.45` | z 方向平滑权重 | 高度变化更平滑 | 更贴原始高度 |
| `path_optimization_max_deviation` | `1.5` | 优化后最大偏离 | 更允许抄近路 | 更保守 |
| `path_optimization_max_z_slope` | `0.45` | z 方向最大坡度 | 爬升/下降更激进 | 高度变化更缓 |
| `path_optimization_min_turn_radius` | `6.0` | 最小转弯半径 | 转弯更圆滑 | 转弯更灵活 |

联动建议：

- 如果 B-spline 容易切进障碍，增大 `path_optimization_data_weight`，减小 `path_optimization_max_deviation`，或减小 `bspline_control_point_spacing`。
- 如果路径太折，增大 `smooth_weight`、`min_turn_radius` 或 `bspline_control_point_spacing`。
- 如果 B-spline 碰撞检查失败但 A* 有路，通常是曲线平滑后切角，需要收紧平滑和偏离参数。

### B-spline 碰撞检查

| 参数 | 默认值 | 作用 | 调大效果 | 调小效果 |
| --- | ---: | --- | --- | --- |
| `bspline_collision_check_enabled` | `true` | 是否检查 B-spline 是否碰动态障碍 | 开启更安全 | 关闭可能曲线切进障碍 |
| `bspline_collision_sample_step` | `0.3` | 碰撞检查采样间隔 | 数值大更快但可能漏检 | 数值小更细但更慢 |
| `bspline_collision_extra_clearance` | `0.1` | B-spline 检查额外安全余量 | 更保守 | 更容易通过检查 |

联动建议：

- 总避障余量约等于 `dynamic_avoidance_obstacle_inflation + safety_distance + bspline_collision_extra_clearance`。
- `bspline_collision_sample_step` 不要大于小障碍物有效尺寸，建议接近或小于 `resolution` 到 `0.5m`。
- 如果频繁提示 B-spline collision failed，可减小 `extra_clearance` 或 `obstacle_inflation`，但先确认感知框没有过大。

### 动态避障与多帧障碍物

| 参数 | 默认值 | 作用 | 调大效果 | 调小效果 |
| --- | ---: | --- | --- | --- |
| `dynamic_avoidance_enabled` | `true` | 是否启用动态障碍物避障 | 开启动态避障 | 关闭只按原路径走 |
| `dynamic_avoidance_replan_period` | `0.5` | 正常动态重规划周期 | 数值大，重规划少、更稳 | 数值小，响应快但抖动/计算重 |
| `dynamic_avoidance_obstacle_stale_time` | `0.5` | 障碍物新鲜度时间 | 容忍短时丢帧 | 旧障碍更快失效 |
| `dynamic_avoidance_obstacle_history_time` | `0.8` | 多帧障碍物历史窗口 | 抗漏检更强，但拖影更多 | 响应快，但可能漏掉断续障碍 |
| `dynamic_avoidance_obstacle_merge_distance` | `0.8` | 多帧障碍物合并距离 | 更容易合并成稳定障碍 | 不容易合并，障碍数量增多 |
| `dynamic_avoidance_immediate_replan_on_collision` | `true` | 当前 B-spline 受威胁时立即重规划 | 响应更快 | 只按周期重规划 |
| `dynamic_avoidance_min_replan_interval` | `0.2` | 最小重规划间隔 | 防止疯狂重规划 | 响应更激进 |
| `dynamic_avoidance_obstacle_inflation` | `0.6` | 动态障碍物膨胀半径 | 更安全但更容易无路 | 更贴近障碍 |
| `dynamic_avoidance_endpoint_clear_radius` | `1.5` | 起点/终点附近放宽障碍影响 | 减少起终点附近卡死 | 起终点附近更保守 |

联动建议：

- 如果雷达偶尔漏检，增大 `obstacle_history_time`；如果障碍拖影导致绕远或停滞，减小它。
- `obstacle_history_time` 必须大于等于 `obstacle_stale_time`，代码里也会强制这样处理。
- 如果收到新障碍后反复重规划，增大 `min_replan_interval` 或 `replan_period`，同时检查 bbox 是否抖动。
- 如果 A* 经常无路径，优先减小 `obstacle_inflation` 和感知框尺寸，而不是关闭避障。

### MPC

| 参数 | 默认值 | 作用 | 调大效果 | 调小效果 |
| --- | ---: | --- | --- | --- |
| `mpc_N` | `60` | MPC 预测步数 | 预测更长，计算更重 | 更快但视野短 |
| `mpc_dt` | `0.02` | 每步时间 | 总预测时间更长 | 控制更细但 horizon 短 |
| `mpc_command_step` | `5` | 取第几步 MPC 输出作为命令 | 命令更前瞻、更激进 | 更贴近当前、更保守 |
| `mpc_max_jerk` | `30.0` | 最大 jerk | 动作更猛 | 更平滑 |
| `mpc_max_acc` | `8.0` | 最大加速度 | 机动更强 | 更稳但跟踪慢 |
| `mpc_max_vel` | `13.0` | 最大速度 | 更快 | 更稳、更容易避障 |
| `mpc_w_pos` | `120.0` | 位置跟踪权重 | 更贴参考路径 | 允许偏离 |
| `mpc_w_vel` | `40.0` | 速度跟踪权重 | 速度更贴参考 | 速度更自由 |
| `mpc_w_terminal_pos` | `150.0` | 末端位置权重 | 预测末端更靠近目标 | 末端约束弱 |
| `mpc_w_jerk` | `0.2` | jerk 惩罚 | 更平滑 | 动作更猛 |
| `mpc_w_jerk_delta` | `0.05` | jerk 变化惩罚 | 控制更顺 | 响应更快 |

预测时间为：

```text
mpc_horizon_time = mpc_N * mpc_dt
默认 60 * 0.02 = 1.2 s
```

联动建议：

- 高速飞行时，`mpc_horizon_time` 要足够长，否则 MPC 看不到足够远的路径。
- `mpc_command_step * mpc_dt` 是实际取命令的前瞻时间。太大可能激进，太小可能响应慢。
- 如果跟踪路径发抖，增大 `mpc_w_jerk / mpc_w_jerk_delta`，或降低 `mpc_max_acc / max_jerk`。
- 如果跟踪明显滞后，增大 `mpc_w_pos / w_terminal_pos`，或适当增大 `max_acc / max_vel`。

### MPC 障碍物约束

| 参数 | 默认值 | 作用 | 调大效果 | 调小效果 |
| --- | ---: | --- | --- | --- |
| `mpc_obstacle_constraints_enabled` | `false` | MPC 内部是否加入障碍约束 | 更强局部避障，但可能求解失败 | 只依赖 A*/B-spline 绕障 |
| `mpc_obstacle_safe_distance` | `1.5` | MPC 障碍安全距离 | 更保守 | 更贴近障碍 |
| `mpc_obstacle_active_distance` | `8.0` | 障碍约束激活距离 | 更早考虑障碍 | 只考虑近障碍 |
| `mpc_obstacle_max_count` | `8` | MPC 最多考虑障碍数 | 更全面但计算更重 | 更快但可能忽略障碍 |
| `mpc_obstacle_check_step` | `5` | 筛选障碍时参考轨迹采样步长 | 更快但筛选粗 | 更细但更慢 |

联动建议：

- MPC 障碍约束建议在 A*/B-spline 避障稳定后再打开。
- `mpc_obstacle_safe_distance` 不应远大于 `dynamic_avoidance_obstacle_inflation + safety_distance`，否则 MPC 会比全局路径更保守，可能出现“路径能走但 MPC 不愿走”。
- 如果 MPC 求解失败频繁，先关闭 `mpc_obstacle_constraints_enabled` 验证是不是约束太紧。

## 需要联合调节的关键组合

### 1. 感知距离与重规划距离

相关参数：

- `lidar_solver`: `roi_max_x`, `fov_prefilter_deg`, `front_fov_deg`
- `AB_planner`: `replan_trigger_distance`, `dynamic_avoidance_replan_period`, `mpc_N`, `mpc_dt`, `mpc_max_vel`

建议：

- `roi_max_x` 应大于无人机在“发现障碍 -> 重规划 -> 执行避障”期间会飞过的距离。
- 高速时要同时增大 `roi_max_x`、`replan_trigger_distance` 或降低 `mpc_max_vel`。

### 2. 障碍物大小与避障安全距离

相关参数：

- `lidar_solver`: `object_filter_max_*`, `cluster_segX_tolerance`, `voxel_leaf_size`
- `AB_planner`: `safety_distance`, `dynamic_avoidance_obstacle_inflation`, `bspline_collision_extra_clearance`, `mpc_obstacle_safe_distance`

建议：

- 如果 bbox 已经偏大，不要再把 `obstacle_inflation` 调得很大。
- 如果 bbox 偏小或雷达只扫到局部表面，可以适当增大 `obstacle_inflation`。
- 总体安全距离越大，A* 越容易无路径，这是正常现象。

### 3. 多帧稳定性与响应速度

相关参数：

- `lidar_solver`: `temporal_smoothing_alpha`, `temporal_smoothing_max_match_distance`
- `AB_planner`: `obstacle_history_time`, `obstacle_stale_time`, `obstacle_merge_distance`, `min_replan_interval`

建议：

- 如果障碍物闪烁：增大 `obstacle_history_time`，降低 `temporal_smoothing_alpha`。
- 如果障碍物已经消失但路径还绕它：减小 `obstacle_history_time`，增大 `temporal_smoothing_alpha`。
- 如果障碍物数量暴涨：减小 `obstacle_merge_distance` 或收紧感知过滤。

### 4. 路径平滑与碰撞检查

相关参数：

- `AB_planner`: `bspline_control_point_spacing`, `path_optimization_*`, `bspline_collision_sample_step`, `bspline_collision_extra_clearance`

建议：

- B-spline 过度切角：减小 `control_point_spacing`，增大 `data_weight`，减小 `max_deviation`。
- 路径太抖：增大 `smooth_weight`，增大 `min_turn_radius`。
- 碰撞检查太慢：增大 `sample_step`，但不要超过小障碍尺寸。

### 5. MPC 跟踪与速度

相关参数：

- `AB_planner`: `mpc_N`, `mpc_dt`, `mpc_command_step`, `mpc_max_vel`, `mpc_max_acc`, `mpc_max_jerk`, `mpc_w_*`

建议：

- 高速时：增大预测时间 `N * dt`，同时确认 `roi_max_x` 和 `replan_trigger_distance` 足够。
- 抖动时：降低 `max_acc / max_jerk`，增大 `w_jerk / w_jerk_delta`。
- 跟踪慢时：增大 `w_pos / w_terminal_pos`，或适当放宽动力学限制。

## 常见现象与调参方向

### bounding box 中心偏

当前代码已经使用 AABB 中心 `(min_pt + max_pt) / 2`。如果仍然偏：

- 检查点云是否只扫到障碍一侧，这是雷达观测限制。
- 适当增大 `dynamic_avoidance_obstacle_inflation` 补偿框偏小。
- 不建议用很大的 `cluster_tolerance` 解决中心偏，它会导致多个物体粘连。

### 地面、墙面、赛道结构被聚成大障碍

优先顺序：

1. 收紧 `roi_min_y / roi_max_y / roi_min_z / roi_max_z`。
2. 降低对应距离段的 `cluster_segX_max_size`。
3. 降低 `object_filter_max_length / max_width / max_volume`。
4. 必要时打开 `ground_segmentation_enabled`。

### 远处点云稀疏，障碍物断裂

优先顺序：

1. 增大远距离段 `cluster_seg2_tolerance / cluster_seg3_tolerance`。
2. 降低远距离段 `cluster_seg2_min_size / cluster_seg3_min_size`。
3. 适当减小 `voxel_leaf_size`。
4. 增大 `obstacle_history_time`，让断续检测在规划端保持一小段时间。

### 频繁重规划，路径抖动

优先顺序：

1. 看 RViz 中 bbox 是否抖动；如果抖，先调 lidar。
2. 增大 `dynamic_avoidance_min_replan_interval`。
3. 增大 `dynamic_avoidance_replan_period`。
4. 减小 `obstacle_history_time` 或 `obstacle_merge_distance`，避免历史障碍膨胀。

### A* 经常无路径

优先顺序：

1. 确认感知框没有把墙/地面误识别成大障碍。
2. 减小 `dynamic_avoidance_obstacle_inflation`。
3. 减小 `safety_distance`。
4. 适当增大 `resolution`，降低地图过细带来的狭窄阻塞。

### B-spline 有路径但 MPC 不走或走得慢

优先顺序：

1. 如果 `mpc_obstacle_constraints_enabled=true`，先临时关掉验证。
2. 降低 `mpc_obstacle_safe_distance`。
3. 增大 `mpc_w_pos / mpc_w_terminal_pos`。
4. 检查 `mpc_max_vel / max_acc / max_jerk` 是否过低。

## 推荐调参流程

1. 只看 `lidar_solver` 输出，确认 `visualized_point_cloud` 和 `detected_bounding_boxes` 合理。
2. 调 ROI，先减少墙面、地面、赛道边界误检。
3. 调 cluster，解决远处断裂或近处粘连。
4. 调 object filter，过滤明显不可能的超大障碍。
5. 再开启 AB_planner 动态避障，调 `obstacle_history_time / obstacle_inflation / replan_period`。
6. 最后调 MPC，使跟踪既不抖也不滞后。

建议每次只改一组参数，并在 RViz 中同时观察：

- `visualized_point_cloud`
- `detected_bounding_boxes`
- `front_fov_obstacle_markers`
- `bspline_path`
- `mpc_prediction_path`
