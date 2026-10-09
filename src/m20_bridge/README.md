# m20_bridge

把 `rs_follow` 的 `/cmd_vel` 接到**云深处山猫 M20 / M20 Pro** 的 `basic_server` TCP/UDP 协议。

## 为什么需要它

M20 的外部运动控制不是 ROS2 `/cmd_vel`，而是 `basic_server` 报文：

```
APDU = 16B头(EB 91 EB 90 | len | id | 0x01 | 7x00) + JSON ASDU
ASDU = {"PatrolDevice": {"Type": t, "Command": c, "Time": "...", "Items": {...}}}
```

| 功能 | Type | Command | 说明 |
|------|------|---------|------|
| 心跳 | 100 | 100 | ≥1Hz，否则机器人不推状态 |
| 使用模式 | 1101 | 5 | `Mode=0` 常规（发轴指令前提）|
| 运动状态 | 2 | 22 | `MotionParam` 枚举须按型号/固件确认；不把示例值当作可用配置 |
| 步态 | 2 | 23 | `GaitParam` 枚举须按型号/固件确认；默认不设置 |
| **轴指令** | **2** | **21** | `X,Y,Z,Roll,Pitch,Yaw∈[-1,1]`，**最大速度比例**，建议 20Hz，需 2s 内同一客户端 |
| 状态 | 1002 | 4 | 10Hz: `Yaw/OmegaZ/LinearX/LinearY` → 里程计 |
| 状态 | 1002 | 6 | 2Hz: `HES/ControlUsageMode/MotionState/Gait` |

机器人是服务端：UDP `10.21.31.103:30000`、TCP `:30001`。

## 桥接做什么

- 心跳 2Hz；默认 `auto_setup=false`，不自动更改机器人的模式、姿态或步态。
- `/cmd_vel`(m/s,rad/s) 经 `dog_adapters.CommandGate` 限幅到 0.3 / 0.15 / 0.5，再转轴比例，20Hz 下发。
- **看门狗**：指令 `watchdog_timeout` 默认 0.3s，状态 `status_timeout` 默认 1.5s，均须有限且大于零；必须有年龄不超过 `status_timeout` 的 `BasicStatus`，且 `HES=0`、`ControlUsageMode=0`。MotionStatus 也使用此新鲜度参数过滤。未知、失联、陈旧、非有限指令均清除缓存并停止；恢复后须新指令。
- 用 1002/4 的速度积分出 `nav_msgs/Odometry`。反馈年龄和积分使用单调时钟；ROS stamp 使用 ROS 时钟；首次、非正或超过 1s 的 dt 不积分。不发布 TF，协方差保守设为大值。

## 运行

```bash
source /opt/ros/humble/setup.bash && source ~/dog_follower/install/setup.bash

# 1) 已通过厂家接口确认常规模式、HES=0、姿态/步态和实际速度比例后再连接真机
# 在 shell 中先填写已确认的标定值；未填写时不运行桥接
: "${M20_FULL_X:?填写已确认的 m/s 比例}"
: "${M20_FULL_Y:?填写已确认的 m/s 比例}"
: "${M20_FULL_YAW:?填写已确认的 rad/s 比例}"
ros2 run m20_bridge bridge_node --ros-args \
  -p transport:=udp -p ip:=10.21.31.103 -p port:=30000 \
  -p cmd_vel_topic:=/rs_follow/cmd_vel -p odom_topic:=/m20/odom \
  -p full_scale_x:="$M20_FULL_X" -p full_scale_y:="$M20_FULL_Y" -p full_scale_yaw:="$M20_FULL_YAW"
# TCP 改为 -p transport:=tcp -p port:=30001；按实际服务器地址调整 ip/port

# 2) 先观察；确认安全后才另行使能跟随。只启动这一个输出适配器
ros2 run rs_follow rs_follow_node --ros-args \
  -p cmd_vel_topic:=/rs_follow/cmd_vel -p odom_topic:=/m20/odom -p active:=false
```

> `rs_follow` 和 `m20_bridge` 在同一台机或同一 LAN 上均可（DDS 跨机即可）。

## `full_scale` 标定（重要）

轴值 1.0 对应机器人最大速度。`full_scale_x` 是"轴值 1.0 对应多少 m/s"。
默认 2.0 m/s / 1.0 m/s / 1.5 rad/s **仅为本地回环测试假设，不是 M20 硬件标定或保证值**。真机必须显式填入厂家确认或安全测试得到的比例值；速度限幅不能弥补比例错误。
硬件标定需在厂商认可的隔离测试环境、硬急停可用且有人监护时进行：对已知非零轴比例测实际速度 `v`，求 `full_scale = v / 轴值`。本次没有执行硬件标定。

## 本地回环自测（不需要真机）

```bash
# 终端1：假 M20 服务器
ros2 run m20_bridge fake_m20_server --ip 127.0.0.1 --port 30000
# 终端2：桥接指向假服务器
ros2 run m20_bridge bridge_node --ros-args -p ip:=127.0.0.1 -p port:=30000
# 终端3：发速度
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.3}}"
# 终端1 应打印: X=0.15 ... Yaw=0.2
```

## 安全

- 桥接观察 `HES`，在 `HES!=0` 或未知状态时发全 0；这是软件门控，不是物理急停的替代，也不证明硬急停或实际制动有效。
- `/rs_follow/estop` (`std_msgs/Bool`) 锁存软件急停；清除后必须收到新速度指令。
- `/rs_follow/action_cmd` 与 Twist/Unitree 共用 `dog_adapters/command_gate.py` 的别名：`softstop/soft_stop/estop/e_stop/stop` 只锁存急停，不发送未经确认的 `MotionParam=2`。`clear/clear_estop/unlock/reset_estop/resume` 清除软件急停，同时丢弃旧指令；不会自动恢复运动或站立，必须收到新速度指令且状态门控通过。
- `standup/stand_up/stand/up` 和 `liedown/lie_down/down` 共用一个处理器。必须显式设置 `setup_enums_confirmed=true` 以及 `stand_motion_state` / `lie_motion_state`（默认 -1），并有新鲜安全状态；否则拒绝。枚举必须由对应型号/固件的实际接口确认，本文中的协议示例不代表确认。
- 自动配置必须显式设置 `auto_setup=true`、`setup_enums_confirmed=true`、`mode=0` 以及确认过的 `motion_state`、`gait`（默认均 -1）；配置用定时器分步发送，不阻塞 ROS 回调。
- `rate_hz`、`heartbeat_hz`、`watchdog_timeout` 和三项 `full_scale_*` 必须为有限正数，否则启动失败。
- UDP 只接受配置服务器 IP/port 的数据。TCP 支持分片/合并帧；非法 magic/fmt/JSON 或 EOF/传输错误终止接收线程并清除就绪状态/命令缓存，不自动重连。
- 接收线程只保留两类最新反馈；ROS 定时器负责消费和发布。退出先取消定时器，发送三次零轴（间隔 0.05s），再停止线程及 socket。
- 建议首次 `active:=false` 观察，随后低速使能；真实安全停机仍须机器人硬急停。

假服务器还支持 `--transport tcp --fragment-size 1`（分片；0 表示合并发送）、`--hes`、`--mode`、`--motion-state`、`--gait`、`--no-feedback`、`--feedback-seconds`（到期停止反馈）和 `--disconnect-seconds`（到期断开）。它跟踪配置请求并反馈状态，不是硬件或 SDK 的替代品。

step2 已完成真实 ROS 2 DDS 与本地 M20 UDP/TCP 回环验收（含状态门控、watchdog 和 TCP 分片/断连）；未连接 M20 硬件，不宣称任何型号/固件的实机兼容性。当前可通过 `ros2 launch dog_adapters follow_stack.launch.py adapter:=m20 robot_config:=/absolute/path/to/config.yaml` 统一启动；必须提供显式部署 YAML，启动 inactive，内部指令为 `/rs_follow/cmd_vel`，只运行这一个输出适配器。step5 真实 launch 的 M20 UDP 无硬件 smoke 已通过，配置/TF 来源、回环示例和边界见[根 README](../../README.md)；launch 不确认实际地址、SDK 权限、状态枚举或比例标定，也不发布硬件 TF。

