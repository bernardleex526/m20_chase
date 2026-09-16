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
- `src/m20_bridge/` — `/cmd_vel` ↔ M20 `basic_server` UDP/TCP 协议桥接（仅 `main`）：
  - 心跳 / 模式切换 / 轴指令 20Hz / 看门狗 / 状态→里程计
- `gazebo/` — Gazebo 无头测试 world（实验记录）

## 快速开始（算法）

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run rs_follow rs_follow_node --ros-args -p input_topic:=/rslidar_points -p active:=false
```

详见 `src/rs_follow/README.md`。

## 快速开始（M20 桥接，`main` 分支）

```bash
ros2 run m20_bridge bridge_node --ros-args -p ip:=10.21.31.103 -p port:=30000
ros2 run rs_follow rs_follow_node --ros-args -p odom_topic:=/m20/odom -p active:=true
```

详见 `src/m20_bridge/README.md`。
