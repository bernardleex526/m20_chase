# 移植指南：更换激光雷达 / 更换机器狗

`rs_follow` 的分层设计让大部分更换工作落在**配置**上，而不是改代码。

```
┌─────────────────────────────────────────────────────────────┐
│ pointcloud_scan.hpp   3D点云 → 2D极坐标扫描   ← 传感器相关   │
├─────────────────────────────────────────────────────────────┤
│ follow_controller.hpp 绑定/跟踪/避障/控制律   ← 传感器无关   │
│ kalman_filter_2d.hpp  卡尔曼滤波            ← 传感器无关   │
├─────────────────────────────────────────────────────────────┤
│ rs_follow_node.cpp    话题/参数/定时下发      ← 接口相关     │
└─────────────────────────────────────────────────────────────┘
```

- 换雷达：主要动 `pointcloud_scan.hpp` 的**输入约定** + 一组参数。
- 换狗：主要动**输出接口**（cmd_vel 话题或其桥接）+ 速度/运动学参数。
- 控制律本身不用动。

---

## 一、更换激光雷达（以 Livox Mid-360 为例）

### 1. 驱动 / 消息格式

| 项目 | RoboSense (现状) | Livox Mid-360 |
|------|------------------|---------------|
| 驱动包 | `rslidar_sdk` | `livox_ros_driver2` |
| 默认话题 | `/rslidar_points` | `/livox/lidar` |
| 默认消息 | `sensor_msgs/PointCloud2` | `livox_ros_driver2/CustomMsg` **或** `PointCloud2` |
| frame_id | `rslidar` | `livox_frame` |

**关键**：`rs_follow` 只接受标准 `PointCloud2`，且需要 FLOAT32 的 `x,y,z` 字段。
Mid-360 驱动要设置输出 `PointCloud2`：

```json
// livox_ros_driver2 config: MID360_config.json / launch 参数
{ "xfer_format": 1 }   // 1 = PointCloud2 (0 = CustomMsg)
```

若必须用 `CustomMsg`，两条路：
- **推荐**：加一个 `CustomMsg → PointCloud2` 转换节点（约 30 行 Python/C++，把 `points[].x/y/z` 拷到 PointCloud2）。
- 或：在 `pointcloud_scan.hpp::projectPointCloud()` 里加对 `CustomMsg` 的重载/分支。

### 2. 仅改参数（无需重新编译逻辑）

在 `config/follow_params.yaml` 里：

```yaml
input_topic: "/livox/lidar"     # ← 换话题
angle_bins: 1440                # Mid-360 可保持
range_min: 0.15                 # Mid-360 近端更近
range_max: 40.0                 # 按需
control_frame: "base_link"      # +x 前、+y 左、+z 上；需测量外参 TF
active: false                  # 硬件 TF 和高度带未实测前保持关闭
height_min: 0.35               # 名义值；必须在 control_frame 中实测
height_max: 2.55
enable_low_band: false
low_height_min: 0.10           # 显式控制帧 z 范围，不从雷达高度推导
low_height_max: 0.30
```

**高度带必须在控制帧中标定**：先测量并发布雷达到 `base_link` 的旋转/平移 TF，再观察变换后地面、目标躯干与低障碍的 z 分布。目标带和低位障碍带分别显式配置；低位带不用于选目标。上述名义值不是任何硬件的标定结果；测量缺失时保持 inactive，低位带关闭。

### 3. 可能需要的小改动

| 情况 | 改动位置 | 做法 |
|------|----------|------|
| 消息不是 XYZ PointCloud2 | `pointcloud_scan.hpp` | 加消息类型分支 / 转换节点 |
| 坐标轴朝向/安装位置不同 | URDF/TF | 测量完整旋转和平移，按点云 stamp 变换到 `control_frame` |
| 点更稀疏、目标簇点变少 | 参数 | 调大 `target_radius`、`lost_frames_timeout`，或减小 `angle_bins` |
| 只输出 LaserScan | `rs_follow_node.cpp` | 订阅 LaserScan 后直接填 `ScanFrame`（`projectPointCloud` 换成一行遍历）|
| 自带地面分割的驱动 | — | 直接吃其输出去地面的点云，`height_*` 可放宽 |

### 4. 其它常见雷达速查

| 雷达 | 话题 | 消息 | 坐标 | 备注 |
|------|------|------|------|------|
| RoboSense Fairy/Airy/Helios | `/rslidar_points` | PointCloud2 | x前 y左 | 已适配 |
| Livox Mid-360 | `/livox/lidar` | CustomMsg/PC2 | x前 y左 | 需 `xfer_format` 出 PC2 |
| Velodyne VLP-16 | `/velodyne_points` | PointCloud2 | x前 y左 | 直接改 `input_topic` |
| Ouster OS0/OS1 | `/ouster/points` | PointCloud2 | x前 y左 | 直接改 `input_topic` |
| Hesai XT16/Pandar | `/hesai/pandar` | PointCloud2 | x前 y左 | 直接改 `input_topic` |

---

## 二、坐标系与 TF（通用正确做法）

默认 `control_frame=base_link`（+x 前、+y 左、+z 上）。节点已通过 `tf2_ros::Buffer` 按每帧点云的非零 stamp 查询旋转与平移，再投影；不回退到最新 TF，不需要 PCL。扫描、目标、marker 与可视化均携带控制帧和源测量时间。

1. 用实测 URDF/静态 TF 描述 `base_link → lidar`；动态 TF 必须覆盖输入测量时间。
2. 在控制帧中实测 `height_min/max`、`low_height_min/max`；安装改变后重新测量，不使用原始传感器高度公式。
3. 标定 `frame_front/back/left/right`（或 `auto_frame` 与机体尺寸/余量）。归一化自遮挡盒在投影和控制器间共享，排除自身点后才取每个 bin 最近回波。
4. RViz Fixed Frame 使用控制帧；绑定点需要非零有效 stamp 与 frame_id，在请求时间查询 TF。绑定事务失败保留已有目标，不自动启用。点云 stamp/布局/TF 故障立即六分量归零并暂停，恢复需新鲜输入与显式重新启用。

`/rs_follow/cloud_viz` 保留变换后的真实源 XYZ（含高度带外点），排除自身点，以 0.1 m 体素去重、最多 6000 点，仅有订阅者时收集并最多 5 Hz 发布。合成点云使用 `control_frame=rslidar` 仅是同帧回环配置，不是硬件外参证据。

当前无硬件 TF/高度带测量，必须保持 `active=false`。适用范围为平地、低速、受控场景的几何跟随，不保证目标身份或复杂地形安全。最终软件证据：四包构建通过、colcon 211 项零错误/失败/跳过、pytest 231 项通过且无跳过、坐标 DDS 8/8、消费者场景 13/13 PASS（含六分量硬停止）、五个 C++ 测试通过；governor 仅为退出 0 的诊断。完整 `follow_acceptance --case all` 7/7 PASS、退出 0，含 crossing 与 clock_pause；交叉用例不证明纯几何身份识别。详见 [最终 step5 报告](../../../.omp/reports/step5.md)；这些是此次清理前已运行的证据，不是硬件验收。

---

## 三、更换机器狗

### 1. 速度接口

| 底盘 | 接口 | 需要做什么 |
|------|------|-----------|
| 通用 ROS 底盘 | Twist / TwistStamped | 使用 `dog_adapters twist_adapter`，显式配置 `output_topic`；Stamped 加 `stamped=true` |
| Unitree Sport API | 官方 `unitree_api/msg/Request` | 使用 `dog_adapters unitree_adapter`；外部官方依赖、型号权限和 arm 要求见根 README |
| M20 / M20 Pro | basic_server UDP/TCP | 使用 `m20_bridge bridge_node`；先确认状态/枚举和标定比例 |
| 其他原生 Twist 底盘 | 厂商驱动速度入口 | 先确认话题、坐标系、权限和反馈；不能仅凭品牌推断支持 |
| 自定义串口狗 | 私有协议 | 写桥接节点 |

统一接线：`ros2 launch dog_adapters follow_stack.launch.py adapter:=twist robot_config:=/absolute/path/to/measured_robot.yaml with_web:=false`。`adapter` 必填且只能是 `m20/twist/unitree_sport`，`robot_config` 必须存在；launch 启动算法 → 内部 `/rs_follow/cmd_vel` → **一个**选择的适配器，`with_web=true` 时另启 Web UI 并使用同一内部指令路径。不要并行启动另一个输出节点。启动强制 `active/auto_select_front/search_enable/recovery_enable/compensate_slip=false`，绑定与使能仍须显式操作。回环命令、Unitree 官方可选依赖和软件 smoke 证据见[根 README](../../../README.md)，M20 见[M20 README](../../m20_bridge/README.md)。

`dog_adapters` 默认速度上限 vx/vy/wz 为 0.3 / 0.15 / 0.5，watchdog 为 0.3s；Twist 输入可输出 Twist 或 TwistStamped。Unitree 默认未 arm，超时/急停会取消 arm，恢复要求重新 arm 和新指令。软件零速度/StopMove 不是硬急停或实际停机确认。AIR/PRO/EDU 的 Sport API 与 DDS 权限须按型号/固件确认；本次仅做真实 DDS 与 M20 本地 UDP/TCP 验收，没有硬件验证。

### 2. 运动学能力

| 狗的能力 | 参数 |
|----------|------|
| 全向（可横移，如四足） | `enable_lateral: true`（默认） |
| 差速（不能横移） | `enable_lateral: false` → 只输出 `vx/wz` |
| 最高速度不同 | `max_linear`、`max_angular` |
| 响应灵敏度不同 | `k_linear`、`k_angular`、`k_lateral`、死区 |

### 3. 尺寸 / 安装变化

| 变化 | 参数 |
|------|------|
| 雷达安装高度/角度 | 测量旋转/平移 TF 与 `control_frame` 中显式目标/低障碍 z 带 |
| 狗体尺寸（自遮挡） | `frame_front/back/left/right` |
| 期望跟随距离 | `follow_dist` |
| 目标大小（人/物） | `target_radius` |

### 4. 安全（换平台务必检查）

- 保持 `active: false` 默认，先看 `/rs_follow/scan`、内部 `/rs_follow/cmd_vel` 和选定适配器的实际输出，再开启。
- 保留 `cmd_timeout` 失联保护与速度治理器 `hardStop` 六分量硬停止。
- 保留适配器单调时钟限幅/看门狗；M20 还要求新鲜 `HES=0`、`ControlUsageMode=0` 状态，默认不自动配置；比例默认值仅供回环测试。实际硬急停和硬件停机效果须另行验证。

### 5. 部署 YAML 与目标机核对

没有仓库提供的已标定真机 YAML。已安装的 `dog_adapters/config/loopback.yaml` 显式 `deployment.nonhardware: true`、`tf_source: loopback_identity`，其中尺寸、高度带、M20 默认比例都是合成测试假设；不能改一个 IP 就作为真机配置。

`nonhardware` 仅是软件声明，不能隔离 DDS 或阻止连接硬件。回环测试使用隔离的 `ROS_DOMAIN_ID`、localhost 通信和测试订阅者，并断开物理机器人控制连接；不得仅凭配置标记判断测试环境安全。

硬件 YAML 顶层填 `deployment.nonhardware: false`，`deployment.tf_source` 命名负责实测外参的 URDF/TF 发布器或来源。此元数据不发布 TF，也不能证明外参正确；应独立启动雷达/TF/里程计来源，并在输入 stamp 验证完整旋转和平移。launch 不制造 identity 硬件外参，只有已处于控制帧的输入可不变换。

节点参数放在 `rs_follow_node.ros__parameters` 和选定 `m20_bridge/twist_adapter/unitree_adapter.ros__parameters` 中；启用 Web 还需 `web_ui.ros__parameters`。必须显式填写输入话题、控制帧、里程计话题；正的 `robot_length/robot_width` 和 `frame_front/back/left/right`；有限且按下界小于上界排列的目标/低位高度带；布尔 `enable_low_band` 和 `auto_frame`。`auto_frame=false` 使用实测 `frame_*`（回环示例采用此模式）；`auto_frame=true` 根据机体尺寸生成遮挡盒，必须另填有限且非负的 `self_occlusion_margin`，不能误以为此时仍使用 `frame_*`。机体遮挡、高度带须来自安装测量，低位带未验收前关闭。Twist 的 `output_topic` 必须与内部指令不同（包括 remap），Stamped 的 `base_frame/stamped` 与控制器契约匹配（默认 `base_frame=base_link`）；Unitree `request_topic` 必填。launch 会覆盖所有参与节点的 `cmd_vel_topic`，不能用 YAML 绕过内部接线。

对 M20 明确填写实际 `transport/ip/port`、反馈 `odom_topic`、已确认的 `full_scale_x/y/yaw`、限幅和 watchdog；默认比例只供回环。保持 `auto_setup=false`；若确需自动配置或站起/趴下，先按实际型号/固件确认 `setup_enums_confirmed`、`mode/motion_state/gait` 与 `stand_motion_state/lie_motion_state`。这些硬件事实不由 launch 校验或推断。Unitree 须确认官方消息/SDK、Sport API 和 DDS 权限，再显式 arm；Twist 须核实横移能力、轴方向、单位、限幅和控制入口权限。未确认参数不能用协议示例或品牌推测补齐。
M20 的 `status_timeout` 默认 1.5s，必须为有限正数；BasicStatus 运动就绪判断及 MotionStatus 新鲜度过滤共用此参数。`watchdog_timeout` 默认 0.3s，独立限制指令年龄。

目标机逐项记录并实际核对：

| 项目 | 必须取得的证据 |
|------|----------------|
| OS / ROS / CPU | 实际系统版本、ROS 发行版、CPU 架构（含 Jetson 模块/JetPack）、原生构建与运行结果；本机 x64 软件结果不证明 arm64 部署 |
| SDK / 固件 / 权限 | 型号、固件与 SDK/消息版本兼容性，控制授权、官方依赖、设备/串口/网络访问权限；M20 状态与枚举逐项确认 |
| DDS / 网络 | `ROS_DOMAIN_ID`、实际 RMW、NIC/路由、雷达地址与端口、机器人端点；跨机发现、消息速率、丢包与输入/输出 QoS 实测 |
| TF / 外参 / 输入 | TF 发布来源、完整实测旋转/平移、测量时间覆盖、同步时钟、点云布局和 frame；实测机体遮挡与控制帧高度带 |
| watchdog / 轴 / 标定 | 指令、观测、云与状态超时；前后/左右/转向符号、单位、横移能力与实际轴比例；断流、急停、恢复须新指令/重新使能 |
| 延迟 / 制动 / 物理停机 | 在目标负载/网络下测量端到端延迟、停车距离和制动响应；按厂家认可的隔离/支撑方式验证硬急停、控制权限撤销与真实停止反馈，再低速使能 |

配置通过只说明软件输入满足契约。尚未在目标机部署、测量外参、执行轴标定或证明物理制动；软件零指令不能代替硬急停。

### 6. 时间基准与回放暂停

估计器、扫描控制和无位姿里程计补偿的 `dt` 来自相邻已接受 `ScanFrame.stamp`，首帧为零；Kalman 接受显式 `dt`，漏检按增量预测，观测更新不重复预测。重复/倒退 stamp 不推进估计器；节点拒绝该云、六分量归零、清扫描/目标/命令缓存并暂停。扫描间隔超过 1s 清除跟踪并要求重新绑定，不能自动选目标；恢复须新鲜输入、显式绑定与使能。

WallTimer 保证 `/clock` 冻结时控制回调继续。云新鲜度、目标观测、DIRECT 指令与输出适配器 watchdog 用单调时间；ROS 消息 stamp 与 TF 查询仍用测量/ROS 时间。仿真下平滑器和搜索/恢复状态机只用 ROS 时间差：相等 tick 输出零且不推进，持续冻结超过 `cmd_timeout` 清缓存并暂停，倒退或大于 1s 跳变立即清缓存/暂停。恢复时间不会恢复旧指令或旧目标，仍须新输入、重新绑定/使能。

step5 已完成真实 launch 无硬件 smoke（[证据](../../../.omp/reports/step5-launch.md)）以及 `follow_acceptance --case all` 退出 0；`/clock` 冻结停止 0.0490s、重复云/新 DIRECT 保持零，时间恢复后重新绑定与使能才恢复运动。该验收观察目标缓存/发布状态，不直接证明不可观测的内部估计器状态；几何目标跟踪也不保证身份。它们不是目标机器或物理停机证据。

---

## 四、改动清单总结

| 目标 | 只改配置 | 需要改代码 |
|------|:--------:|:----------:|
| 换同为 XYZ PointCloud2 的雷达 | ✅ `input_topic`、测量 TF、控制帧高度带、`range_*` | — |
| 换消息格式特殊（Livox CustomMsg / LaserScan） | — | `pointcloud_scan.hpp` 或加转换节点 |
| 雷达有安装旋转/平移 | ✅ 实测 URDF/TF，节点已按输入 stamp 变换 | — |
| 换 Twist / TwistStamped 接口的狗 | ✅ `twist_adapter` 输入/输出话题、stamped、速度/运动学参数 | — |
| 换官方 Unitree Sport 或 M20 basic_server | 使用现有适配器并确认权限/枚举/比例/状态 | 其他私有接口仍需实现新桥接 |
| 换狗尺寸/雷达高度 | ✅ `frame_*`、`height_*`、`follow_dist` | — |

**核心原则**：控制律 (`follow_controller.hpp`) 与传感器、底盘解耦；适配工作集中在
「输入投影」和「输出桥接」两端。
