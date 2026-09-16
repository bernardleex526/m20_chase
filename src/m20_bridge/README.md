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
| 运动状态 | 2 | 22 | `MotionParam:1` 站立 / `17` RL |
| 步态 | 2 | 23 | `0x1001` 基础 / `0x3002` 平地 |
| **轴指令** | **2** | **21** | `X,Y,Z,Roll,Pitch,Yaw∈[-1,1]`，**最大速度比例**，建议 20Hz，需 2s 内同一客户端 |
| 状态 | 1002 | 4 | 10Hz: `Yaw/OmegaZ/LinearX/LinearY` → 里程计 |
| 状态 | 1002 | 6 | 2Hz: `HES/ControlUsageMode/MotionState/Gait` |

机器人是服务端：UDP `10.21.31.103:30000`、TCP `:30001`。

## 桥接做什么

- 心跳 2Hz；启动时自动切 `mode=0` + 站立 + 步态
- `/cmd_vel`(m/s,rad/s) → 轴比例，20Hz 下发
- **看门狗**：0.5s 无指令 或 硬急停(HES) 或 非常规模式 → 发全 0
- 用 1002/4 的速度积分出 `nav_msgs/Odometry` 发到 `odom_topic`（供 rs_follow 做打滑补偿/惯性系滤波）

## 运行

```bash
source /opt/ros/humble/setup.bash && source ~/dog_follower/install/setup.bash

# 1) 启动桥接（连真机）
ros2 run m20_bridge bridge_node --ros-args \
  -p transport:=udp -p ip:=10.21.31.103 -p port:=30000 \
  -p cmd_vel_topic:=/cmd_vel -p odom_topic:=/m20/odom \
  -p full_scale_x:=2.0 -p full_scale_y:=1.0 -p full_scale_yaw:=1.5

# 2) 启动跟随（把 odom 指到桥接的输出）
ros2 run rs_follow rs_follow_node --ros-args \
  -p cmd_vel_topic:=/cmd_vel -p odom_topic:=/m20/odom -p active:=true
```

> `rs_follow` 和 `m20_bridge` 在同一台机或同一 LAN 上均可（DDS 跨机即可）。

## `full_scale` 标定（重要）

轴值 1.0 对应机器人最大速度。`full_scale_x` 是"轴值 1.0 对应多少 m/s"。
标定：让狗在空地，桥接直接发固定轴值（可临时 `ros2 topic pub /cmd_vel ...`），
用卷尺/秒表测实际速度 `v`，则 `full_scale_x = v / 轴值`。
默认 2.0 m/s / 1.0 m/s / 1.5 rad/s，按实机调整。

## 本地回环自测（不需要真机）

```bash
# 终端1：假 M20 服务器
ros2 run m20_bridge fake_m20_server --ip 127.0.0.1 --port 30000
# 终端2：桥接指向假服务器
ros2 run m20_bridge bridge_node --ros-args -p ip:=127.0.0.1 -p port:=30000
# 终端3：发速度
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.6}, angular: {z: 0.3}}"
# 终端1 应打印: axis X=+0.300 ... Yaw=+0.200
```

## 安全

- 狗侧**硬急停**始终有效；桥接在 `HES=1` 时发全 0
- 看门狗：指令超时、模式不对 → 发全 0
- 建议 `rs_follow` 首次 `active:=false` 观察，再低速使能（`max_linear:=0.3`）
- 轴指令要求"2 秒内同一客户端"：本桥接以同一 socket 20Hz 持续发送，满足要求
