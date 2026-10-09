# rs_follow

通用 **RoboSense 3D 激光雷达点云跟随** ROS 2 包。

把 3D 点云投影成 2D 极坐标扫描，绑定/锁定一个目标点云簇，按「方位 + 距离」计算并下发
`geometry_msgs/Twist` 到 `/cmd_vel`，给机器狗（或任意底盘）执行。算法控制律参考
[jie_deamon](https://github.com/6-robot/jie_deamon)（MIT），并从 2D `LaserScan` 扩展为直接适配
RoboSense `PointCloud2`。

## 数据流

```
/rslidar_points (PointCloud2)
        │  按云 stamp 的 TF → control_frame → 共享自遮挡盒 → 高度带/方位最近回波
        ▼
FollowController
   ├─ 目标获取: 自动选正前方最近点 或 手动绑定(/clicked_point)
   ├─ 目标跟踪: 目标半径内点簇质心 + 卡尔曼平滑
   ├─ 避障: 势场法排斥力 + 急停 + 减速
   └─ 控制律: 距离误差→linear.x  方位误差→angular.z  走廊→linear.y
        ▼
/cmd_vel (Twist)
```

默认 `control_frame=base_link`（+x 前、+y 左、+z 上）。每帧按非零源 stamp 查询 TF 的旋转和平移，不借用最新 TF；扫描、目标和可视化使用控制帧与源测量时间。`height_min/max` 与 `low_height_min/max` 是该帧中的显式 z 范围，低位带仅用于障碍，默认关闭，不从传感器高度推导。投影和控制器共享归一化的 `frame_front/back/left/right` 自遮挡盒；先排除自身回波，再取各 bin 最近点，避免机体回波掩盖外部目标。

`/rs_follow/cloud_viz` 是变换后的真实源 XYZ 点云，不是虚拟扫描补出的点；保留带外点、排除自身点，0.1 m 体素去重、最多 6000 点，仅有订阅者时收集并最多 5 Hz 发布。RViz Fixed Frame 使用 `control_frame`。硬件外参和高度带尚未实测，保持 `active=false`；合成点云设 `control_frame=rslidar` 只适用于同帧软件回环。

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
ros2 run rs_follow rs_follow_node
```

或一键（`with_lidar:=true` 会同时拉起雷达，不启动 RViz）：

```bash
ros2 launch rs_follow follow.launch.py with_lidar:=true
```

## 选取 / 绑定目标

三种方式：

1. **自动（仅专门测试显式开启）**：`auto_select_front=true` 可选正前方最近点簇；部署默认关闭，几何跟踪不保证行人身份。
2. **RViz 手动绑定**：在 RViz 里用 **Publish Point** 发布带非零测量 stamp 和 frame_id 的 `/clicked_point`；节点按点击 stamp 将点变换到控制帧，再吸附到新鲜扫描回波。RViz Fixed Frame 设为 `base_link`（或实际 `control_frame`）。
3. **命令行绑定**（示例 stamp 必须替换为本次有效测量时间；零 stamp 拒绝）：

```bash
ros2 topic pub --once /rs_follow/bind_target geometry_msgs/msg/PointStamped \
  "{header: {stamp: {sec: 42, nanosec: 123}, frame_id: base_link}, point: {x: 2.0, y: 0.3, z: 1.0}}"

# 清除绑定并暂停，不自动跟随他人
ros2 topic pub --once /rs_follow/clear_target std_msgs/msg/Bool "{data: true}"
```

推荐通过 `/rs_follow/bind` 服务确认绑定，不用旧 `/target` 消息猜测本次是否成功：

```bash
ros2 service call /rs_follow/bind rs_follow_interfaces/srv/BindTarget \
  "{point: {header: {stamp: {sec: 42, nanosec: 123}, frame_id: base_link}, point: {x: 3.0, y: 0.0, z: 1.0}}}"
# success=true / reason=OK 后仍需显式启用
ros2 topic pub --once /rs_follow/enable std_msgs/msg/Bool '{data: true}'
```

失败原因固定为 `NO_SCAN|STALE_SCAN|BAD_POINT|TF_UNAVAILABLE|NO_RETURN`；绑定按事务提交，失败保留原目标且不启用运动。空 frame、零/非法 stamp、非有限坐标、自身点均拒绝；跨 frame 必须在请求 stamp 有 TF，不使用最新 TF 回退。
点云布局、stamp 或 TF 失败立即发布六分量零速、清缓存并暂停；后续有效云不能自动恢复，需新鲜有效输入与显式重新 enable。清除绑定、目标观测超时后仍需重新绑定和显式启用。

### 软件停止与看门狗

`/rs_follow/estop` (`std_msgs/Bool`, reliable/volatile/depth 1) 的 true 锁存并立即发布六分量零速；
false 仅解除锁存，仍暂停，不站立、不恢复缓存。必须重新显式 enable 和提供新命令才能运动。
这是软件停止，不能替代随身物理急停或机器人本体 watchdog。
硬停止、未启用、无有效云、云龄超过 `cmd_timeout=0.5s`、DIRECT 指令龄超过
`direct_cmd_timeout=0.3s`、非有限输出均清平滑/命令缓存并输出零；接收 watchdog 使用单调时间。
DIRECT 模式仍为 0、FOLLOW 为 1；非法模式忽略，非有限直接命令拒绝并清缓存。
真实观测龄超过 `target_observation_timeout=0.5s` 暂停并清目标，Kalman 预测不刷新观测龄；重新选择目标后显式启用。
部署默认 `active/auto_select_front/search_enable/recovery_enable/compensate_slip=false`。
`/rs_follow/control_state` String JSON 只含 `active,mode,estop,target_valid`，来自控制定时器。

软件回环验收入口（不连接机器狗）：`ros2 run rs_follow follow_acceptance.py --case all`。
已覆盖 DIRECT 断流、绑定、hard_stop、软件停止、点云断流、交叉遮挡与暂停仿真时钟；最终 7/7 PASS，退出 0。交叉用例验证观测超时停止、不自动换目标，不证明纯几何跟踪能识别合并或门限内交叉的人身份。

最终软件验证：四包构建通过；colcon 211 项零错误/失败/跳过；pytest 231 项通过、无跳过；坐标 DDS 验收 8/8；消费者场景 13/13 PASS（含 `EMERGENCY_STOP` 六分量归零）；五个 C++ 安全/自动选择/投影/绑定/时间测试通过，governor 仅为退出 0 的诊断。完整 `follow_acceptance --case all` 7/7 PASS、退出 0；DIRECT 0.3093 s、丢失观测 0.5123 s、点云断流 0.5108 s、软件停止 0.0006 s、冻结时钟归零 0.0490 s。真实桌面与 390px 手机浏览器流程通过。详见 [最终 step5 证据与边界](../../.omp/reports/step5.md)。这些是此次清理前已运行的软件证据，不代表硬件验证。

适用边界是平地、低速、受控场景的几何目标跟随；不保证目标身份，不证明行人识别、复杂地形安全、实体制动或全工况通过。

## Web 控制台（唯一 WebUi）

`web_ui.py` 提供 Canvas2D **点云俯视图**与手动控制；未实现的导航入口和重复目标 UI 已移除。
这是实际 XYZ 回波的俯视投影，不是可旋转三维场景：`+x` 向上、`+y` 向左，每格 1m，按 z 高度蓝→红着色。
需要 ROS 环境中的 `python3-websockets`；节点、TF 和 `/rs_follow/cloud_viz` 数据链必须先就绪。

```bash
source install/setup.bash
ros2 run rs_follow web_ui.py --ros-args -p bind_host:=127.0.0.1 -p http_port:=8080 -p ws_port:=8890
# 本机浏览器打开 http://127.0.0.1:8080；回环默认空令牌也须点击“连接”完成认证
```

### 选择、确认、跟随、停止

1. 输入访问令牌并点击“连接”；令牌仅保存在页面内存，不进入 URL 或持久存储。第一条 WS 消息为认证。
   第一位认证操作者取得租约，其他客户端显示只读；页面每 100ms 发送 heartbeat，租约超过 0.5s 或断线则后端暂停并归零。
   重连不恢复运动，需要重新绑定确认 / 显式启动；租约不代表厂商底盘仲裁权限。
2. 在“点云跟随”画布点击 / 轻触（按下至释放位移 ≤8 CSS px），选平面距离 ≤0.8m 的最近显示回波；黄色圆圈只是候选。
   点击“绑定目标”携带该回波原始 xyz、frame、stamp 调用 `/rs_follow/bind`，等待明确服务结果。
   页面显示 `OK|NO_SCAN|STALE_SCAN|BAD_POINT|TF_UNAVAILABLE|NO_RETURN`；失败不能冒充成功，绑定不自动运动。
   候选超过 0.5s 会禁用绑定并提示重新点选；缺失 / 过期 / 非法点云清空显示并禁止选择。
3. 服务返回 OK 且节点目标有效后，点击“开始跟随”：请求 FOLLOW，等待节点确认模式及暂停，再显式 enable。
   无绑定、新鲜数据不足、软件停止锁存时不可启动。页面 active / mode / estop / target_valid 只显示 ROS 节点确认值，不根据按钮猜测。
4. “暂停”归零并 disable；“取消 / 清除目标”先暂停再清目标。全页常驻“软件停止”请求 estop=true 并暂停，
   “解除停止”只解除锁存，不自动启动、不自动站起。**网页不能代替随身物理急停**。
   已取消但尚未完成的绑定服务请求会保持失效安全暂停，直到服务完成并处理迟到结果，不允许自动 enable。
   若服务端消失、请求无法完成，须先停止节点并清除实际目标，再重启 WebUi 恢复控制门面；不能以重启绕过暂停或自动恢复运动。

### 手动控制与部署边界

- “手动控制”先点击“启用手动”，等待 DIRECT 和 active 确认，只有按住控件才发送 20Hz 指令。
  摇杆向上为 +x、向左为 +y；滑块左拖为正 yaw（左转）。前向 / 横向 / 转向上限分别为 0.3m/s、0.15m/s、0.5rad/s。
  Pointer capture、松开、cancel、lost capture、窗口 blur、页面隐藏或切页均归零；没有按住时不循环发送运动指令。
  站起 / 趴下为显式动作请求，不代表厂商已执行反馈。
- 状态通过 text JSON，点云通过独立二进制 PC01：ASCII4、LE uint32 seq、int32 sec、uint32 nanosec、uint32 count、
  uint16 frame 字节长度、UTF-8 frame、count×3 LE float32 xyz。单帧自带 header；前端严格拒绝非法版本、长度、UTF-8 和非有限坐标。
- 非回环暴露须设置至少 32 字节随机 `auth_token` 和明确 `allowed_origins`，使用 HTTPS/WSS 反向代理或 SSH 隧道。
  `/api/config` 提供 WS 端口及可选 `ws_url`（用于反向代理）；HTTP API 使用 Bearer 认证，不能替代 WS 操作者心跳租约。
  即使仅在回环地址使用，调用受保护 HTTP API 也须配置非空 `auth_token` 并发送 `Authorization: Bearer <token>`；
  回环 WS 的空令牌认证不构成有效 HTTP Bearer 凭据。远程暴露仍须满足至少 32 字节随机令牌要求。
- 390px 手机页面可用且允许页面缩放。限定**平坦地面、低速、受控环境**；纯几何点簇跟随不保证人员身份，
  交叉 / 遮挡可能误认，软件停止送达也不证明实体制动距离或本体 watchdog。真机仍需物理急停、权限与轴向标定验收。

软件浏览器回环已观察到实际点云、OK 绑定后 FOLLOW 非零输出、软件停止归零且解除后暂停、DIRECT 左拖正 yaw、cancel / blur 归零；
390px 已验证操作者完整流程：实际 60 点云、OK 绑定、FOLLOW 非零输出、停止归零、解除仍暂停；地面回波绑定返回 NO_RETURN 且启动禁用。
手机截图为 `.omp/reports/step4-mobile-{bound,follow,release,rejected}.png`；第二客户端只读且无横向溢出。
桌面画面见 `.omp/reports/step4-desktop-cloud.png`、`step4-desktop-bound.png`、`step4-desktop-follow.png`；
完整证据及尚未验收项见 [step4 报告](../../.omp/reports/step4.md)。上述仅为软件证据，不代表机器狗或硬件安全验收；最终测试重跑结果以报告为准。


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
| 发布 | `/rs_follow/target` | `geometry_msgs/PointStamped` | 当前锁定目标（control_frame，源扫描 stamp）|
| 发布 | `/rs_follow/target_marker` | `visualization_msgs/Marker` | 目标标记 |
| 发布 | `/rs_follow/status` | `std_msgs/String` | `NO_TARGET / TRACKING_AUTO / TRACKING_MANUAL / EMERGENCY_STOP` |

## 关键参数

见 `config/follow_params.yaml`。常用：

| 参数 | 默认 | 说明 |
|------|------|------|
| `control_frame` | base_link | 所有几何计算的控制帧；按输入 stamp 查询测量外参 TF |
| `height_min` / `height_max` | 0.35 / 2.55 | 控制帧中的目标 z 带；名义迁移值，硬件须实测 |
| `enable_low_band` / `low_height_min` / `low_height_max` | false / 0.10 / 0.30 | 控制帧中显式低障碍带；不作为目标，未标定保持关闭 |
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
| `compensate_slip` | false | 实测标定后才允许按实际/指令比例补偿 |
| `slip_min_ratio` / `slip_max_ratio` | 0.3 / 2.5 | 比例估计的夹取范围 |
| `slip_filter_alpha` | 0.1 | 比例估计一阶低通 |
| `max_linear_cmd` / `max_angular_cmd` | 1.5 / 2.0 | 补偿后指令的安全上限 |
| `enable_lateral` | true | 是否输出 `linear.y`（机器狗横移）。非全向底盘请设 false |
| `governor_d_hard` / `governor_emergency_floor` | 0.25 / 0.08 | 方向净空硬停止距离与全向净空硬停止下限 (m)；须按实机外廓和制动测量标定 |
| `auto_select_front` | false | 专门测试可显式开启；部署必须人工绑定 |
| `angle_bins` | 1440 | 方位分辨率（0.25°）|

## 上机器狗（后续）

1. 把 `cmd_vel_topic` 指到狗的速度话题（默认 `/cmd_vel`）。
2. 测量雷达相对 `base_link` 的旋转和平移，通过 URDF/TF 发布外参；确认实际点云 stamp 对应的 TF 可用。
3. 在控制帧中实测目标带、低障碍带与地面范围，并标定共享自遮挡盒；不套用传感器高度公式。缺少这些测量时保持 `active=false`、低位带关闭。
4. 根据底盘能力调整 `enable_lateral`、`max_linear`、`k_*`。
5. 真机首测务必保持 `active=false`，用 `/rs_follow/scan` 与 `/cmd_vel` 观察，再逐步开启。

## 说明

- 跟随距离、方位均以 **control_frame** 水平面计算；`bearing=0` 为控制帧 +x 正前方。
- 目标丢失超过 `lost_frames_timeout` 帧后解除绑定；若开启自动选点会重新选正前方目标。
- 控制指令在定时器（`control_rate_hz`）里下发；超过 `cmd_timeout` 无新点云立即清缓存、暂停并归零，恢复需新鲜输入与显式重新 enable。

## 速度、加速度与平滑

控制分两级：

1. **控制器**（`follow_controller.hpp`）每帧算出「期望速度」，并做死区**滞回**，避免在目标距离
   附近反复启停。
2. **输出平滑器**（`cmd_smoother.hpp`，在控制定时器里按真实 dt 运行）把期望速度变成实际下发速度：
   - **加速度限幅**：`max_linear_accel` / `max_angular_accel`（普通状态）；
   - **一阶低通**：`cmd_filter_alpha`，抑制残留抖动；
   - **硬停止例外**：速度治理器 `hardStop` 触发时**跳过限幅瞬时六分量归零**（安全优先）。
   - 失联、观测超时或关闭走最高优先级停止门，瞬时清缓存归零，不允许平滑器残留运动。

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
