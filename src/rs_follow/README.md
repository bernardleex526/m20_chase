# rs_follow

通用 **RoboSense 3D 激光雷达点云跟随** ROS 2 包。

把 3D 点云投影成 2D 极坐标扫描，绑定/锁定一个目标点云簇，按「方位 + 距离」计算并下发
`geometry_msgs/Twist` 到 `/cmd_vel`，给机器狗（或任意底盘）执行。算法控制律参考
[jie_deamon](https://github.com/6-robot/jie_deamon)（MIT），并从 2D `LaserScan` 扩展为直接适配
RoboSense `PointCloud2`。

## 数据流

```
/rslidar_points (PointCloud2)
        │  projectPointCloud(): 高度带切片 + 方位分bin取最近 → 虚拟2D扫描(1440 bins)
        ▼
FollowController
   ├─ 目标获取: 自动选正前方最近点 或 手动绑定(/clicked_point)
   ├─ 目标跟踪: 目标半径内点簇质心 + 卡尔曼平滑
   ├─ 避障: 势场法排斥力 + 急停 + 减速
   └─ 控制律: 距离误差→linear.x  方位误差→angular.z  走廊→linear.y
        ▼
/cmd_vel (Twist)
```

## 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/dog_follower
colcon build --symlink-install
source install/setup.bash
```

## 运行

先启动雷达驱动（若未启动）：

```bash
source ~/rslidar_ws/install/setup.bash
ros2 run rslidar_sdk rslidar_sdk_node
```

再启动跟随节点（默认 `active=false`，安全起见不会动）：

```bash
source ~/dog_follower/install/setup.bash
ros2 run rs_follow rs_follow_node --ros-args -p active:=true
```

或一键（`with_lidar:=true` 会同时拉起雷达，不启动 RViz）：

```bash
ros2 launch rs_follow follow.launch.py active:=true with_lidar:=true
```

## 选取 / 绑定目标

三种方式：

1. **自动（默认）**：`auto_select_front=true`，未绑定目标时自动选正前方 ±`auto_front_fov_deg`
   内、`auto_select_max_range` 内最近的点。适合「正前方有且只有行人」的场景。
2. **RViz 手动绑定**：在 RViz 里用工具栏 **Publish Point**，点击点云中想跟随的目标 →
   发到 `/clicked_point`。节点会把点击点吸附到最近的扫描回波上并锁定。
   RViz 的 Fixed Frame 设为 `rslidar`。
3. **命令行绑定**：

```bash
ros2 topic pub --once /rs_follow/bind_target geometry_msgs/msg/PointStamped \
  "{header: {frame_id: rslidar}, point: {x: 2.0, y: 0.3, z: 0.0}}"

# 清除绑定，回到自动选点
ros2 topic pub --once /rs_follow/clear_target std_msgs/msg/Bool "{data: true}"
```

## 话题

| 方向 | 话题 | 类型 | 说明 |
|------|------|------|------|
| 订阅 | `/rslidar_points` | `sensor_msgs/PointCloud2` | 雷达点云（可配 `input_topic`）|
| 订阅 | `/clicked_point` | `geometry_msgs/PointStamped` | RViz Publish Point 选点 |
| 订阅 | `/rs_follow/bind_target` | `geometry_msgs/PointStamped` | 程序化绑定目标 |
| 订阅 | `/rs_follow/clear_target` | `std_msgs/Bool` | 清除目标 |
| 订阅 | `/rs_follow/enable` | `std_msgs/Bool` | 使能/暂停跟随 |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 |
| 发布 | `/rs_follow/scan` | `sensor_msgs/LaserScan` | 投影出的 2D 扫描（RViz 可视化）|
| 发布 | `/rs_follow/target` | `geometry_msgs/PointStamped` | 当前锁定目标（雷达系）|
| 发布 | `/rs_follow/target_marker` | `visualization_msgs/Marker` | 目标标记 |
| 发布 | `/rs_follow/status` | `std_msgs/String` | `NO_TARGET / TRACKING_AUTO / TRACKING_MANUAL / EMERGENCY_STOP` |

## 关键参数

见 `config/follow_params.yaml`。常用：

| 参数 | 默认 | 说明 |
|------|------|------|
| `height_min` / `height_max` | -0.4 / 1.8 | 点云高度带（滤地面/天花板）。雷达装在 0.5m 高时，行人躯干约在 0~1.3m |
| `follow_dist` | 1.0 | 期望跟随距离 (m) |
| `target_radius` | 0.6 | 目标点簇半径 (m) |
| `k_linear` / `k_angular` | 2.5 / 1.5 | 线速度/角速度比例系数 |
| `max_linear` / `max_angular` | 0.9 / 1.0 | 速度上限 |
| `linear_hysteresis` / `angular_hysteresis` | 0.03 | 死区滞回，抑制在目标距离附近的开/关抖动 |
| `max_linear_accel` / `max_angular_accel` | 0.8 / 1.5 | 加速度上限（m/s²、rad/s²），按控制周期限幅 |
| `cmd_filter_alpha` | 0.6 | 指令一阶低通系数，0=关闭 |
| `k_integral` / `integral_limit` | 0.3 / 1.0 | 积分项（消除移动目标稳态误差）与抗饱和上限 |
| `enable_kalman` | true | 目标卡尔曼滤波 |
| `kalman_q` / `kalman_r` | 0.1 / 0.05 | 过程/观测噪声 |
| `kalman_gate` | 6.0 | 马氏距离门限（σ），剔除离群点簇 |
| `kalman_r_ref_points` | 80 | 标称点簇点数，R 按 n_ref/n 自适应 |
| `filter_in_world` | true | 在惯性系滤波（用里程计补偿机体旋转）|
| `odom_topic` | `/odom` | 机器人里程计；置空则回退到指令积分 |
| `compensate_slip` | true | 按 `实际/指令` 比例放大速度，抵消打滑 |
| `slip_min_ratio` / `slip_max_ratio` | 0.3 / 2.5 | 比例估计的夹取范围 |
| `slip_filter_alpha` | 0.1 | 比例估计一阶低通 |
| `max_linear_cmd` / `max_angular_cmd` | 1.5 / 2.0 | 补偿后指令的安全上限 |
| `enable_lateral` | true | 是否输出 `linear.y`（机器狗横移）。非全向底盘请设 false |
| `apf_emergency` | 0.35 | 障碍急停距离 (m) |
| `auto_select_front` | true | 未绑定时自动选正前方目标 |
| `angle_bins` | 1440 | 方位分辨率（0.25°）|
| `flip_x` / `flip_y` | false | 若点云坐标轴朝向与机体不一致时翻转 |

## 上机器狗（后续）

1. 把 `cmd_vel_topic` 指到狗的速度话题（默认 `/cmd_vel`）。
2. 确认雷达坐标系相对机体的朝外参：若装反了，用 `flip_x/flip_y` 或加 TF 修正。
3. 标定 `height_min/height_max`（雷达离地高度 + 行人高度范围）。
4. 根据底盘能力调整 `enable_lateral`、`max_linear`、`k_*`。
5. 真机首测务必保持 `active=false`，用 `/rs_follow/scan` 与 `/cmd_vel` 观察，再逐步开启。

## 说明

- 跟随距离、方位均以**雷达坐标系**水平面计算；`bearing=0` 为雷达正前方。
- 目标丢失超过 `lost_frames_timeout` 帧后解除绑定；若开启自动选点会重新选正前方目标。
- 控制指令在定时器（`control_rate_hz`）里下发；超过 `cmd_timeout` 无新点云则平滑减速到零（失联保护）。

## 速度、加速度与平滑

控制分两级：

1. **控制器**（`follow_controller.hpp`）每帧算出「期望速度」，并做死区**滞回**，避免在目标距离
   附近反复启停。
2. **输出平滑器**（`cmd_smoother.hpp`，在控制定时器里按真实 dt 运行）把期望速度变成实际下发速度：
   - **加速度限幅**：`max_linear_accel` / `max_angular_accel`（普通状态）；
   - **一阶低通**：`cmd_filter_alpha`，抑制残留抖动；
   - **急停例外**：`apf_emergency` 触发时**跳过限幅瞬时归零**（安全优先）。
   - 失联/丢目标/关闭时按加速度上限**缓降**到零，而不是硬切。

实测（合成点云，控制 20Hz）：

| 指标 | 平滑前 | 平滑后 |
|------|--------|--------|
| 静止→跟随 最大线加速度 | 11.3 m/s² | **0.80 m/s²** |
| 方位阶跃 最大角加速度 | 12.6 rad/s² | **1.50 rad/s²** |
| 保持距离时 cmd 抖动 (std vx) | 0.105 m/s | **0.053 m/s** |
| 急停 | 瞬时归零 | 瞬时归零（安全设计，不受限幅） |

调参建议：机器狗比轮式更怕顿挫，可把 `max_linear_accel` 调到 `0.5`、`max_angular_accel` 调到
`1.0`；若转向仍抖，把 `cmd_filter_alpha` 调小（如 0.4）；若跟随反应迟钝，则先调大 `k_*`。

## 仿真跟随效果（闭环，含打滑）

`follow_sim_visualize.py` 的稳态站位误差（目标跟随距离 1.0m）：

| 工况 | 无打滑 | 打滑 0.7 + 补偿 |
|------|:---:|:---:|
| approach（静目标接近）| 3.2 cm | 3.7 cm |
| chase（绕圈跟踪）| 10.7 cm | 12.2 cm |
| stopgo（走走停停）| 10.4 cm | — |
| obstacle（障碍急停后恢复）| 11.5 cm | — |
| turn（扫向侧后方）| 18.1 cm | 18.1 cm |
| lost（遮挡 3s 后重现）| 45.4 cm | 46.7 cm |

打滑补偿关闭时，0.7 打滑的 turn 会退化到 **88.5 cm**；开启后回到 18.1 cm。

## 测试脚本（SSH/无界面环境）

**1. 点云状态检查**（查看雷达点云健康度，无需 RViz）：

```bash
source /opt/ros/humble/setup.bash && source ~/dog_follower/install/setup.bash
ros2 run rs_follow pointcloud_status.py --topic /rslidar_points --duration 5
```

输出：帧率、每帧点数、有效点比例、x/y/z 范围、水平距离分布、z 高度直方图、正前方点数。

**2. 跟随场景测试**（合成点云确定性验证各状态下 `/cmd_vel`）：

```bash
ros2 run rs_follow follow_scenario_test.py --duration 1.5
```

覆盖场景：远目标前进、到达距离保持、过近后退、左/右转、目标在后方原地转、
近障碍急停、走廊横向修正、自动选点、目标丢失。脚本会自己拉起一个 `rs_follow_node`
（监听 `/test_points`）并把结果打成表格。

## 移植到其它雷达/机器狗

见 [`docs/PORTING.md`](docs/PORTING.md)：换 Livox Mid-360 等雷达、换机器狗（含非 Twist
接口的桥接）、坐标系/TF 处理、以及需要改哪些文件的清单。

FastLIO2 / FastLIVO2 建图定位方案与验证口径见 [`docs/SLAM_FASTLIO2_PLAN.md`](docs/SLAM_FASTLIO2_PLAN.md)。

## 可视化仿真（肉眼确认）

`follow_sim_visualize.py` 是**闭环仿真**：虚拟机器人由真实 `rs_follow_node` 驱动，
仿真器按发布的 `/cmd_vel` 积分位姿、从机器人视角生成点云再回灌给节点，输出：

- `~/dog_follower/rs_follow_sim/<实验>.gif` — 俯视动画（点云、机器人朝向、指令箭头）
- `~/dog_follower/rs_follow_sim/<实验>.png` — 静态多面板（轨迹、cmd 时间序列、距离、方位）

```bash
ros2 run rs_follow follow_sim_visualize.py --experiment chase     # 追移动目标
ros2 run rs_follow follow_sim_visualize.py --experiment obstacle  # 障碍急停后恢复
```

`control_smoothness_check.py` 量化过渡时刻的线/角加速度与稳态抖动，用于验证平滑效果。

## 物理 + 光线投射仿真（无头，无需 Gazebo 渲染）

`follow_pysim.py` 是自包含的**物理+光线投射**仿真：差速动力学（一阶响应/打滑/延迟）+
3D 光线投射激光（地面/墙/圆柱目标/真实遮挡/测距噪声/丢点）+ 可选四足步态俯仰，闭环驱动
真实 `rs_follow_node`，输出同款 GIF/PNG。

```bash
ros2 run rs_follow follow_pysim.py --all                    # open / corridor / gait
ros2 run rs_follow follow_pysim.py --all --slip 0.7         # 打滑
ros2 run rs_follow follow_pysim.py --scenario corridor --gait 4
```

场景：`open`（开阔跟随）、`corridor`（走廊）、`gait`（4° 步态俯仰）。
结果（站位误差，目标距离 1.0m）：open **8.1cm**、corridor **13.4cm**、gait **8.4cm**。
