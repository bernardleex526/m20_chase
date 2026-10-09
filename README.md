# m20_chase

RoboSense Airy 3D 激光点云**跟随算法** + 云深处**山猫 M20 / M20 Pro** 运控桥接。

## 分支说明

| 分支 | 内容 |
|------|------|
| `main` | 完整方案：跟随算法 + M20 `basic_server` 桥接 |
| `algo-only` | 纯算法适配：仅 `rs_follow`（不含 M20 桥接） |

## 目录

- `src/rs_follow/` — 通用 RoboSense 点云跟随：
  - 3D 点云 → 2D 极坐标投影（高度带滤地面）
  - 目标绑定（RViz `/clicked_point` 或 `/rs_follow/bind_target`）+ 卡尔曼跟踪
  - 惯性系滤波（用 `/odom` 补偿机体旋转）+ 打滑补偿
  - 势场避障 / 急停；距离→vx、方位→wz、走廊→vy
  - 输出 `/cmd_vel`（Twist），加速度 + jerk 限幅
  - 测试脚本：点云状态、场景测试、平滑度、运动学仿真、物理+光线投射仿真
- `src/dog_adapters/` — 共用单调时钟安全门控的 ROS 2 输出适配：`twist_adapter`（Twist / TwistStamped）和 `unitree_adapter`（官方 Sport Request）。
- `src/m20_bridge/` — `/cmd_vel` ↔ M20 `basic_server` UDP/TCP 协议桥接（仅 `main`）：
  - 心跳 / 模式切换 / 轴指令 20Hz / 看门狗 / 状态→里程计
- `gazebo/` — Gazebo 无头测试 world（实验记录）
- `simulation/gazebo/` — [可复现的 CHAMP 动力学仿真入口](simulation/gazebo/README.md)与[最终 9/9 场景观测摘要](simulation/gazebo/VALIDATION.md)；通用模型证据，不代替 M20/Go2 真机验收。

## 快速开始（算法）

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run rs_follow rs_follow_node --ros-args -p input_topic:=/rslidar_points -p active:=false
```

详见 `src/rs_follow/README.md`。

默认 `control_frame=base_link`（+x 前、+y 左、+z 上）：按点云非零 stamp 查询旋转/平移 TF，再按控制帧显式目标/低障碍高度带投影。投影和控制器共享归一化机体自遮挡盒，先排除自身点再取最近回波。硬件外参与高度带未实测，保持 `active=false`、低位带关闭；合成 `rslidar` 同帧配置仅用于软件回环。

绑定点必须带有效 stamp/frame，按请求时间 TF 变换并事务提交，失败保留已有目标且不自动使能。点云布局/stamp/TF 故障立即六分量归零、清缓存并暂停；恢复要求新鲜输入与显式重新启用。`/rs_follow/cloud_viz` 发布真实变换后源 XYZ（含带外点、排除自身点），订阅者驱动，最多 5 Hz、0.1 m 体素、6000 点。适用范围是平地、低速、受控场景的几何跟随，不保证目标身份。

## 统一启动（follow_stack）

`dog_adapters/follow_stack.launch.py` 启动一个跟随节点、**一个**显式选择的输出适配器，以及可选 Web UI。`adapter` 必填，只接受 `m20`、`twist`、`unitree_sport`；`robot_config` 必须指向已有 YAML；`with_web` 默认为 `true`，只接受 `true/false`。无需 Web 时显式设置 `with_web:=false`。不要同时运行手动输出节点或另一个 stack。

以下配置只供无硬件回环；Twist/Unitree 输出只能连接测试订阅者，不能连接机器人：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dog_adapters follow_stack.launch.py adapter:=twist \
  robot_config:=$(ros2 pkg prefix dog_adapters)/share/dog_adapters/config/loopback.yaml \
  with_web:=true
```

同一回环 YAML 可选 `adapter:=m20`（另开终端运行 `ros2 run m20_bridge fake_m20_server --ip 127.0.0.1 --port 30000`）或 `adapter:=unitree_sport`（先 source 官方 `unitree_api` 工作空间，见下文）。配置输入 `/rslidar_points`、里程计 `/m20/odom`、控制帧 `rslidar`；合成点云须带非零递增 stamp 且处于 `rslidar`。这是同帧软件回环，不是硬件外参。没有启动雷达驱动、点云生成器或 TF 发布器。Web 示例仅绑定 `127.0.0.1`，HTTP 8080、WebSocket 8890。

YAML 使用 `rs_follow_node` 和所选 `m20_bridge` / `twist_adapter` / `unitree_adapter` 的 `ros__parameters` 节；`with_web=true` 还要求 `web_ui.ros__parameters`。顶层 `deployment.nonhardware` 必须显式为布尔值，`deployment.tf_source` 必须命名实测 URDF/TF 发布源；`loopback_identity` 只允许显式无硬件配置。该字段只是来源记录，不生成或验证外参。运行时仍按点云 stamp 查询实际 TF；已在控制帧的云无需额外变换。

`rs_follow_node` 必须显式提供非空 `input_topic/control_frame/odom_topic`、正数 `robot_length/robot_width/frame_front/frame_back/frame_left/frame_right`、有限且有序的 `height_min/max` 与 `low_height_min/max`、布尔 `enable_low_band` 和 `auto_frame`。`auto_frame=false` 使用实测 `frame_*`；`auto_frame=true` 则根据机体尺寸生成遮挡盒，必须另填有限且非负的 `self_occlusion_margin`。回环示例显式 `auto_frame=false`。Twist 必须提供与内部输入不同的 `output_topic`，Unitree 必须提供 `request_topic`；无硬件 M20 必须显式填写 loopback `ip`。其余节点参数沿用各节点接口，不能把默认值当成硬件标定。真机配置、平台清单和 TF 要求见 [PORTING](src/rs_follow/docs/PORTING.md)。

启动强制 `active/auto_select_front/search_enable/recovery_enable/compensate_slip=false`；跟随、适配器和 Web 的 `cmd_vel_topic` 强制接到内部 `/rs_follow/cmd_vel`。先验证输入、绑定和物理停机，再显式使能；Unitree 还须显式 arm，M20 仍受安全状态门控。inactive 启动不构成硬件安全证明，配置校验也不确认 SDK 权限、轴比例或制动效果。

## 时间与暂停策略

估计器和扫描控制使用相邻已接受扫描的源 stamp 差值 `dt`，首帧为零；不从回调墙钟或回放倍速推导。重复/倒退 stamp 不推进估计器，节点归零、清扫描/目标/命令缓存并暂停；大于 1s 的扫描间隔清除跟踪，禁止自动重新选目标，恢复要求新鲜输入、显式重新绑定和使能。

控制发布使用 WallTimer，云、目标观测、DIRECT 指令及适配器 watchdog 的年龄使用单调时钟。`use_sim_time` 下平滑器与搜索/恢复状态机只随 ROS 时间推进；相邻相等 `/clock` tick 输出零且不积分，持续冻结超过 `cmd_timeout` 后清缓存并暂停，倒退或大于 1s 的 ROS 时间跳变立即清缓存/暂停。`/clock` 停止不会停止 watchdog；恢复也必须有新鲜输入、重新绑定与显式使能。消息和 TwistStamped 的 stamp 仍使用 ROS/测量时间。

## 手动诊断接线

以下命令用于单节点诊断，不能与 `follow_stack` 并行运行。先将算法输出隔离到 `/rs_follow/cmd_vel`，保持 `active:=false`，检查输出和硬急停后才使能：

```bash
ros2 run rs_follow rs_follow_node --ros-args \
  -p cmd_vel_topic:=/rs_follow/cmd_vel -p active:=false

# 原生 Twist 控制器：output_topic 必须是实际控制器入口，不能与输入相同（含 remap）
ros2 run dog_adapters twist_adapter --ros-args -p output_topic:=/robot/cmd_vel
# 若控制器要求 TwistStamped，改用同一节点并加：
# -p stamped:=true -p base_frame:=base_link
```

`dog_adapters` 默认输入 `/rs_follow/cmd_vel`，输出 20Hz；`max_vx=0.3` m/s、`max_vy=0.15` m/s、`max_wz=0.5` rad/s，`watchdog_timeout=0.3` s，`lateral=true`（差速底盘设为 false）。非有限指令、超时和软件急停清除缓存；时钟门控使用单调时钟，不随 ROS 仿真时间暂停。TwistStamped 的 stamp 仍用 ROS 时钟。Twist 节点无 arm 服务，启动无指令时输出零；不支持站起/趴下动作。

### Unitree 官方 ROS 2 消息接口

`unitree_api` 是可选外部依赖，仅 Unitree 节点需要。先从 [官方 unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) 构建真正的 `unitree_api` 及其官方 ROSIDL/DDS 依赖，并 source 该工作空间；缺少官方消息包时启动失败，不使用替代消息、SDK shim 或 fallback：

```bash
source /path/to/official_unitree_workspace/install/setup.bash
source install/setup.bash
ros2 run dog_adapters unitree_adapter --ros-args -p request_topic:=/api/sport/request
# 另一个终端（同样 source），确认安全且软件急停已清除后显式 arm
ros2 service call /unitree_adapter/arm std_srvs/srv/SetBool '{data: true}'
# arm 后必须在 0.3s 内收到新速度；取消使能：
ros2 service call /unitree_adapter/arm std_srvs/srv/SetBool '{data: false}'
```

节点启动/未 arm 时发送 StopMove (1003)，arm 后新指令发送 Move (1008)，JSON `x/y/z` 分别为 vx/vy/wz。超时、非法指令或急停均 StopMove 并取消 arm；恢复须重新 arm 和新指令。显式 `standup/stand_up/stand/up`、`liedown/lie_down/down/lie` 动作仅在已 arm 且无急停时发送 StandUp (1004) / StandDown (1005)，清除旧速度；不自动站起、不发送 Damp。

Twist、Unitree 与原 M20 桥接共用 `dog_adapters/command_gate.py` 的动作契约，均订阅 `/rs_follow/estop`（Bool，可用 `estop_topic` 改名）和 `/rs_follow/action_cmd`（String）：`softstop/soft_stop/estop/e_stop/stop` 锁存软件急停，`clear/clear_estop/unlock/reset_estop/resume` 或 Bool false 清除。停止与解除均清除旧速度，解除不会自动恢复运动或站立；Twist/M20 必须收到新速度指令，Unitree 还须显式重新 arm，且 arm 后收到新指令。退出发送零 Twist、StopMove 或 M20 零轴。这是**软件请求，不是物理急停，也不是停机反馈保证**；必须保留机器人硬急停。AIR/PRO/EDU 的 SDK、Sport API、网络/DDS 访问权限因型号、固件和授权而异；不能从消息可发布推断该机器有权限或兼容。

### M20 basic_server

使用 `m20_bridge bridge_node`，并显式加 `-p cmd_vel_topic:=/rs_follow/cmd_vel` 与上述算法接线。安全状态、确认枚举、比例标定和 UDP/TCP 命令见 [M20 README](src/m20_bridge/README.md)；不要并行启动其他输出适配器。

### 已完成验证的边界

最终软件验证：四包构建通过，colcon 211 项零错误/失败/跳过，pytest 231 项通过且无跳过；消费者场景 13/13、坐标 DDS 8/8、五个 C++ 测试与完整 `follow_acceptance --case all` 7/7 PASS（退出 0）。`src/dog_adapters/test/step2_acceptance.py` 真实 ROS 2 DDS Twist、TwistStamped、官方 Unitree Request 和 M20 UDP/TCP 本地服务器 smoke 通过。Unitree 消息来自官方上游提交 `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88`，使用官方 Humble `rosidl_dds` 构建。真实桌面及 390px 手机浏览器完整交互通过。详见 [最终 step5 报告](.omp/reports/step5.md)，此前检查点保留在历史报告中。**没有连接机器人硬件**；这些是此次清理前的软件证据，不证明任何 AIR/PRO/EDU 或 M20 固件兼容性、物理制动或标定值。

step5 真实 `follow_stack` 软件 smoke 已通过：三种适配器分别独占启动、inactive 零输出、必填配置/TF 来源与话题回环拒绝、Twist/TwistStamped 限幅、官方 Unitree arm/Move/StopMove、M20 UDP 状态门控/比例/watchdog，以及可选 Web HTTP/WS 启动。Web 启动检查不代表全部交互验收；详见 [launch smoke](.omp/reports/step5-launch.md)。`follow_acceptance --case all` 已退出 0：crossing 观测超时 0.5084s 后暂停且不自动选目标；实际 `/clock` 冻结停止 0.0490s，重复 stamp 云与新 DIRECT 指令保持零，恢复时间并重新绑定/使能后运动恢复。几何跟踪不保证身份，缓存失效是外部观测，不是内部估计器状态证明。全部为无硬件软件证据，未执行目标机器人部署、外参/轴比例标定或物理停机验收。
