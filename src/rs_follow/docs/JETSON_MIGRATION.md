# m20_chase / rs_follow 迁移 NVIDIA Jetson NX 16GB 与「通用机器人」适配评估

> 本文回答用户提出的第二个追加问题：
> 「m20_chase 的该仓库是否可以迁移至 nvidia nx 16gb 的上位机，然后适配不同机器人的 sdk 和桥接节点来达到通用所有的机器人」
>
> 结论先行：**可以迁移，而且几乎是「零改动迁移」**；「通用所有机器人」在**架构上是成立的**，但**不是靠一份代码跑遍所有机器人**，而是靠本文第 4 节的「三段式适配层」——算法内核不动，只在**输入投影**和**输出桥接**两端各写一个薄适配器。第 6 节给出工作量与优先级。

---

最终软件验证：四包构建通过、colcon 211 项零错误/失败/跳过、pytest 231 项通过且无跳过、坐标 DDS 验收 8/8、消费者场景 13/13 PASS（含六分量硬停止）、五个 C++ 测试通过；governor 仅为退出 0 的诊断。完整 `follow_acceptance --case all` 7/7 PASS、退出 0，包含 crossing 与 clock_pause。证据与边界见 [最终 step5 报告](../../../.omp/reports/step5.md)；这些是此次清理前的软件证据。本文既有 WSL 性能/传感器高度数据是历史测量，不能替代 Jetson 或机器人实测。硬件 TF 与高度带未实测，保持 `active=false`；几何跟随适用于平地、低速、受控场景，不保证身份。

## 0. 结论速览

| 问题 | 结论 | 依据 |
|---|---|---|
| rs_follow 能否跑在 Jetson NX 16GB 上？ | **可以，CPU 余量极大** | 实测 131072 点 @10 Hz 时 **3.0 % 单核**、RSS **28 MB**（第 2 节） |
| 有没有 x86 专属代码需要重写？ | **没有，一行都不用改** | 全仓库无 SIMD/intrinsic/绝对路径（第 3.1 节） |
| 是否依赖 PCL / OpenCV / Eigen / CUDA？ | 投影与 TF 变换不需要 PCL / OpenCV / CUDA | 当前增加 tf2/tf2_ros 与绑定接口依赖，见第 3.2 节 |
| 会不会被 JetPack 版本卡住？ | **Xavier NX 会被卡住，Orin NX 不会**（关键差异，第 1 节） | Xavier NX 最高 JetPack 5.1.7 = Ubuntu 20.04；Humble 官方 deb 只发 22.04 |
| 能否统一适配所有机器狗？ | **架构可行**，接口是标准 `/cmd_vel`（Twist） | 第 4 节三段式分层 |
| 通用化的真实瓶颈在哪？ | **不在算法，在传感器外参 + 各厂商 SDK 的私有协议** | 第 4.3、5 节 |

---

## 1. 首要决策点：Xavier NX 还是 Orin NX —— 这决定你要做多少工作

「Jetson NX 16GB」在 NVIDIA 产品线里对应**两代完全不同的模块**，它们的 Ubuntu 版本不同，而这直接决定 ROS 2 Humble 的安装难度。**这是整个迁移里唯一一个「选错就多花两天」的点。**

| | **Jetson Xavier NX 16GB**（2019，Volta） | **Jetson Orin NX 16GB**（2022，Ampere） |
|---|---|---|
| 最高 JetPack | **5.1.7**（JetPack 6.x **不支持** Xavier NX） | **6.2.x** |
| 根文件系统 | **Ubuntu 20.04** | **Ubuntu 22.04** |
| 内核 | 5.10 | 5.15 |
| CUDA | 11.4 | 12.6 |
| ROS 2 Humble 官方 deb | **不支持**（22.04 专属） | **支持**（`ros-humble-desktop` 直接 apt） |
| AI 算力 | 21 TOPS | 100 TOPS（Super 模式更高） |
| 对本项目的适配成本 | 中：需源码编译 ROS 或走 Docker | **低：apt 直接装，和本机 WSL 环境一致** |

**事实来源**：NVIDIA [JetPack Archive](https://developer.nvidia.com/embedded/jetpack-archive) 显示 JetPack 6.0–6.2.3 的支持列表为「Jetson AGX Orin Series, Jetson Orin NX Series, Jetson Orin Nano Series」，**不含 Xavier NX**；而 JetPack 5.1–5.1.7 的支持列表包含「Jetson Xavier NX series」。JetPack 5.1 页面明确写「an **Ubuntu 20.04** based root file system」；JetPack 6.2 页面写「an **Ubuntu 22.04** based root file system」。ROS 2 Humble 安装页明确写「Deb packages for ROS 2 Humble Hawksbill are currently available for **Ubuntu Jammy (22.04)**」，目标平台为 `amd64, arm64`。

> **给决策者的一句话**：如果你手上是 **Xavier NX 16GB**，请预算半天到一天在「让 ROS 2 Humble 在 20.04 上跑起来」；如果你能选 **Orin NX 16GB**，那这一步是 20 分钟的 apt 安装，并且**和本次验证用的 WSL 环境完全同构**（都是 22.04 + Humble），`REPORT.md` 的复现脚本可以原样搬过去。

### 1.1 Xavier NX 上跑 Humble 的三条路（按推荐度排序）

| 路线 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| **A. 官方 arm64 Docker 镜像** | JetPack 5.1.7 上装 Docker + nvidia-container-runtime，用 ROS 官方 arm64 Humble 镜像（底层 22.04） | 不用污染宿主；22.04 二进制可直接用 | 需配置串口/网口直通（机器人通信通常走串口或 UDP，`--network host` + `--device` 可解决） |
| **B. 源码编译 Humble** | 在 20.04 上按 [Ubuntu source build](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) 编译 | 原生性能，无容器开销 | 首次编译数小时；后续 `colcon` 更新需维护 |
| **C. 升级到 Orin NX** | 换模块 | 一劳永逸，22.04 原生 + 100 TOPS | 硬件成本 |

**推荐 A**：本项目算法是**纯 CPU、无 GPU 依赖**（第 2 节实测），容器化的性能损失可忽略，而隔离性收益很大。注意本项目最终要在真机上跑串口/UDP 桥接，容器务必用 `--network host`。

### 1.2 Orin NX 上的路径（最省事）

```bash
# JetPack 6.2 (Ubuntu 22.04) 上，与本机 WSL 完全同构
sudo apt update && sudo apt install -y ros-humble-ros-base ros-dev-tools
# 然后就是 REPORT.md 第 5 节的复现命令，一字不改
```

> Orin NX 16GB 还额外解锁 **Super Mode**（JetPack 6.2 起，Orin NX 16GB 支持 10W/15W/25W/40W/MAXN SUPER），AI TOPS 提升最多 70%。但**对本项目无意义**——因为 rs_follow 根本不用 GPU（见下节），省电模式反而更合适（机器狗续航）。

---

## 2. 性能实测：Jetson 跑得动吗？—— 余量非常大

本机 WSL 环境：AMD Ryzen 5 5600（6 核 12 线程，但 WSL 只暴露了 8 个逻辑 CPU）、15.9 GB 内存、**无 GPU**。

用 `scripts/bench_cloud.py` 合成真实尺寸的 PointCloud2 直接喂给 rs_follow 节点（绕过 Gazebo，因此测到的**纯粹是算法本身的 CPU**），用 `/proc/<pid>/stat` 的 utime+stime 差分计算：

| 点云规模 | 对应真实雷达 | rs_follow_node CPU | RSS |
|---|---|---|---|
| 6 144 点 @10 Hz | Gazebo 射线传感器（本次测试所用） | **0.8 %** 单核 | 24.7 MB |
| 32 768 点 @10 Hz | RoboSense RS-Helios-16 级别 | **1.4 %** 单核 | 25.6 MB |
| 65 536 点 @10 Hz | RoboSense RS-Ruby lite 级别 | **1.9 %** 单核 | 26.4 MB |
| 131 072 点 @10 Hz | 高密度 128 线级别 | **3.0 %** 单核 | 28.0 MB |

**关键结论**：
1. **算力完全不是瓶颈**。即便喂 13 万点/帧，也只吃掉 3 % 的单核。Jetson Xavier NX 是 6 核 Carmel ARM，Orin NX 是 8 核 Cortex-A78AE，单核性能虽弱于 Ryzen 5，但按 5–8 倍保守折算，**13 万点场景下也只占约 15–25 % 单核**，仍有巨大余量。
2. **内存完全不是瓶颈**。16 GB 模块上这个节点只占 **28 MB**，连零头都算不上。真正的内存消耗者是建图/SLAM（FAST-LIO2 之类）和视觉模型，不是跟随算法。
3. **算法的成本随点数近似线性**（6k→131k 点数涨 21 倍，CPU 只从 0.8 % 涨到 3.0 %，约 3.75 倍）——因为 `pointcloud_scan.hpp` 的高度带过滤在投影前就把绝大部分点丢弃了，剩下的按角度分箱取最近值。这是**很健康的复杂度特征**。

> 运行配套实测（含 Gazebo，供对比）：`gzserver` 占 **~13 % 单核 / 1.36 GB RSS**，是整条链路里最重的进程。这也反过来印证：**真机上没有 Gazebo，Jetson 的负载会比本机测试时轻得多。**

### 2.1 一个需要真机确认的隐性成本：点云解码

上面的数字**不含**雷达驱动把 UDP 包解成 PointCloud2 的开销。RoboSense 的 `rslidar_sdk` 在 arm64 上有官方支持，但：

- **建议开启 `rslidar_sdk` 的 `use_vlan`/`dense_points` 等选项前先实测**；
- 如果雷达是 **Livox Mid-360**，官方 `livox_ros_driver2` 在 arm64 上需要源码编译，且默认发 `CustomMsg`（非 PointCloud2），需按 `docs/PORTING.md` 设 `xfer_format: 1` 转成 PointCloud2；
- **真机首次上车时先只跑驱动 + `pointcloud_status.py`**，确认 `/rslidar_points` 稳定 10 Hz 且点数在预期范围，再启动跟随节点。

---

## 3. 代码可移植性审计（这是「零改动」结论的依据）

### 3.1 无任何架构相关代码

对 `src/rs_follow/` 全量 grep：

- `x86` / `SSE` / `AVX` / `_mm_` / `immintrin` / `__m128` / `arm_neon` → **无匹配**；
- `/dev/` / `/proc/` / `/sys/` 绝对路径 → **无匹配**。

即：**没有一段代码需要为 arm64 改写**。

### 3.2 依赖极简（这是它能在 Jetson 上轻松跑起来的根本原因）

历史 WSL 被测版本的 `src/rs_follow/CMakeLists.txt` 声明如下；这是历史依赖快照，当前坐标迁移还使用 `tf2`、`tf2_ros` 和 `rs_follow_interfaces`，应以当前清单为准：

```cmake
ament_cmake rclcpp sensor_msgs geometry_msgs nav_msgs visualization_msgs std_msgs
```

上述头文件与单文件可执行产出描述属于历史版本。当前节点通过 tf2 查询刚体变换，投影直接处理 XYZ，无需 PCL 点云转换，也不引入 OpenCV/CUDA。

> 这一点**极其关键**：PCL 和 OpenCV 在 Jetson 上是最大的依赖痛点（版本冲突、编译数小时、与 JetPack 自带的 OpenCV 打架）。本仓库天然绕开了这两个坑。**这是它作为一个「可上车」项目的最大优势之一。**

### 3.3 编译标志

```cmake
cmake_minimum_required(VERSION 3.8)
CMAKE_CXX_STANDARD 17
-Wall -Wextra -Wpedantic -O2
```

C++17 在 JetPack 5（GCC 9.4）和 JetPack 6（GCC 11.4）上都原生支持。`-O2` 无架构假设。

### 3.4 许可证

`package.xml` 写明 **`<license>MIT</license>`**，`maintainer email="jetson@example.com"` —— 维护者字段本身就是 `jetson`，说明**作者原本就是按 Jetson 上车设计的**。MIT 许可对商用友好（对比：配套交付物 `OPENSOURCE_FOLLOWING_REPOS.md` 中不少同类项目是 GPL/AGPL 或无 License）。

---

## 4. 「通用所有机器人」：架构可行，但需要三段式适配层

### 4.1 为什么说架构上可行

`docs/PORTING.md`（**该文件在 main 与 algo-only 两个分支上完全相同，`git diff main algo-only -- src/rs_follow/docs/PORTING.md` 为空**）已经把设计意图写清楚了，核心原则逐字是：

> 控制律 `follow_controller.hpp` 与传感器、底盘**解耦**，适配工作集中在「输入投影」与「输出桥接」两端。

这个判断是对的，而且它直接对应一个干净的三段式结构：

```
[1] 输入投影层           [2] 算法内核（不动）        [3] 输出桥接层
雷达驱动 → PointCloud2 →  rs_follow_node            → /cmd_vel (Twist) → 厂商 SDK/桥接 → 机器狗
  ↑ 换雷达只改这里        follow_controller.hpp       ↑ 换底盘只改这里
                          pointcloud_scan.hpp
                          kalman_filter_2d.hpp
                          cmd_smoother.hpp
```

**证据**：算法内核的输入是一个标准 `sensor_msgs/PointCloud2`，输出是一个标准 `geometry_msgs/Twist`，且内核**完全不感知底盘存在**——它只是发布速度指令。这是「通用化」的**必要条件**，本项目已经具备。

### 4.2 第 3 段（输出桥接）：各厂商的实际情况

这是「通用」这个词真正被考验的地方。按接口类型分三类：

| 类型 | 代表机器人 | 适配做法 | 工作量 |
|---|---|---|---|
| **A. 原生 Twist / TwistStamped** | 具有标准 ROS 2 速度控制入口的底盘 | 现有 `dog_adapters twist_adapter`；显式填写 output_topic/stamped | 配置及现场确认 |
| **B. 私有 SDK / 私有协议** | **DeepRobotics M20**、其他私有接口 | M20 已有 `m20_bridge`；其他协议仍需实现桥接 | 按接口评估 |
| **C. 官方非 Twist 消息** | 支持官方 Unitree Sport API 的型号 | 现有 `dog_adapters unitree_adapter`；必须 source 官方 unitree_api | 依赖、权限及现场确认 |

当前可用 `dog_adapters follow_stack.launch.py` 显式选择 `adapter:=m20/twist/unitree_sport`，必须提供 `robot_config` YAML，默认 inactive，仅启动一个输出适配器；可选 `with_web:=true`。命令、配置契约、arm 服务和可选依赖见[根 README](../../../README.md)。step5 真实 launch 的三种输出及 Web 启动无硬件 smoke 已通过。AIR/PRO/EDU 的 SDK/Sport API/DDS 权限必须按型号、固件和授权核实，不能据此表宣称所有型号兼容；软件 DDS 与 M20 本地 UDP/TCP 证据不证明 Jetson 部署或硬件验证。

**M20 当前实现要点**（详细参数见[M20 README](../../m20_bridge/README.md)）：

- **协议**：严格 16 字节 APDU 头 + JSON ASDU；TCP 支持分片/合并，非法 magic/fmt/JSON、EOF 或传输错误取消接收并清除就绪和旧指令，不自动重连；UDP 只接受配置服务器来源。
- **量纲归一化**：`[-1,1]` 轴值是速度比例，不是 m/s；`full_scale_x/y/yaw` 默认 2.0 / 1.0 / 1.5 **仅作回环测试假设**，实机必须确认/标定并显式设置。固定 vx/vy/wz 限幅 0.3 / 0.15 / 0.5 不能代替比例标定。
- **看门狗和状态**：指令 `watchdog_timeout` 默认 0.3s，状态 `status_timeout` 默认 1.5s，均须有限且大于零；采用单调时钟。只在新鲜 BasicStatus 明确 `HES=0`、`ControlUsageMode=0` 时接受运动，MotionStatus 也按 `status_timeout` 过滤。未知/失联/陈旧状态、非有限速度或急停清除旧指令；恢复须新指令。软件零轴不是物理停机保证。
- **保守配置**：默认 `auto_setup=false`；自动配置需显式确认 `setup_enums_confirmed=true`、`mode=0`、实际 motion_state/gait 枚举。站起/趴下还需显式 stand_motion_state/lie_motion_state，均默认 -1；不得把协议示例枚举当成已确认值。
- **反馈与线程**：接收线程只保存最新 BasicStatus/MotionStatus，ROS 定时器消费并发布 odom；反馈年龄及积分用单调时钟，ROS stamp 用 ROS 时钟；首次、非正及超过 1s 的 dt 不积分，不发布 TF。退出取消定时器、尝试三次零轴（间隔 0.05s），再停止线程/socket。

> 比例标定及硬急停/实际制动验证应在厂商认可的安全测试环境中单独执行。本次未执行硬件标定；`REPORT.md` 中既有跟随误差是历史仿真/实验结果，不能作为当前机器人比例或实机兼容性证据。

> **历史文档引用说明**：旧版 PORTING 提到的 `sw01_dog_follower` 属于其他仓库，当前指南已改为本仓库 `dog_adapters` / `m20_bridge`。原有 `SLAM_FASTLIO2_PLAN.md` 悬空引用不属于本次输出适配工作。下方保留的分支对比和 REPORT 数据是此前审阅时的历史记录，不是 step2 后工作树的包数量/内容快照。

### 4.3 第 1 段（输入投影）：换雷达的坑比换底盘更深

当前默认 `control_frame=base_link`（+x 前、+y 左、+z 上），点云按非零源 stamp 查询完整旋转/平移 TF 后才进入投影，不能以最新 TF 代替测量时刻。`height_min/max` 与 `low_height_min/max` 是控制帧中的显式 z 范围；低位带仅用于障碍且默认关闭。标定须观察变换后的地面、躯干与低障碍，不能用传感器离地高度推导当前带。

**历史 WSL 记录**：`follow_test.sdf` 曾用雷达桅杆及传感器帧高度带 `-0.60 / 1.50`（旧默认 `-0.40 / 1.80`）排除地面。这些数据保留用于解释 `REPORT.md` 的历史结果，不是当前 base_link 高度带或硬件外参标定值。

换雷达的具体做法：

| 雷达 | 话题 | 做法 |
|---|---|---|
| RoboSense（当前） | `/rslidar_points` | 默认，`input_topic` 即可 |
| Livox Mid-360 | `/livox/lidar` | 需 `xfer_format: 1` 输出 PointCloud2 |
| Velodyne | `/velodyne_points` | 直接改 `input_topic` |
| Ouster | `/ouster/points` | 直接改 `input_topic` |
| Hesai | `/hesai/pandar` | 直接改 `input_topic` |

TF 查询与逐点旋转/平移已实现，无需 PCL。通过测量 URDF/TF 配置外参，投影与控制器使用同一归一化自遮挡盒，先滤自身点再取 bin 最近点。绑定点必须有有效非零 stamp/frame，在请求时间变换后按事务提交；失败保留目标，不自动使能。云布局、stamp 或 TF 失败立即六分量归零、清缓存并暂停，恢复要求新鲜输入与显式重新启用。

真实源点可视化 `/rs_follow/cloud_viz` 使用控制帧和源 stamp，包含带外点、排除自身点；0.1 m 体素、最多 6000 点、订阅者驱动、最多 5 Hz。合成 `rslidar` 同帧配置仅用于软件回环；硬件外参与高度带缺失时必须保持 inactive。

---

## 5. 迁移执行清单（可直接照着做）

### 阶段 1：环境（半天）
1. 确认模块是 **Xavier NX 还是 Orin NX**（第 1 节，这决定后续路径）。
2. 装 ROS 2 Humble：Orin NX → `apt`；Xavier NX → Docker 或源码编译。
3. `git clone` + 切 `algo-only` 分支（本次验证用的分支，**排除了 M20 私有协议，纯算法**）。
4. 按当前 package/CMake 依赖安装并 `colcon build`；本次四包构建通过不等于 Jetson 已实测。

### 阶段 2：雷达接入（1 天）
5. 装雷达驱动（RoboSense → `rslidar_sdk`；Livox → `livox_ros_driver2`）。
6. **先只验证传感器**：`ros2 topic hz /rslidar_points` 看频率，跑 `pointcloud_status.py` 看点云健康度。
7. 测量完整安装旋转/平移并发布 TF；在 `control_frame=base_link` 中实测目标带、低障碍带、地面和机体自遮挡盒，未完成前保持 `active=false`、低位带关闭。
8. 确认每帧源 stamp 的 TF 可用，检查控制帧扫描/真实源点可视化和带 stamp 绑定；同帧合成回环不能替代该检查。

### 阶段 3：桥接接入（1–3 天，取决于机器人类型）
9. 判断目标机器人属于 A/B/C 哪一类（第 4.2 节）。
10. A 类：配置现有 `twist_adapter`，确认控制入口需要 Twist 还是 TwistStamped，避免输入/输出 remap 成同一话题。
11. Unitree：配置官方依赖与权限后使用 `unitree_adapter`，显式 arm、新速度指令和 0.3s watchdog；M20：使用 `m20_bridge`，确认安全状态、枚举和比例。其他 B/C 接口仍需桥接，务必包含：
    - 物理量到厂商指令的量纲映射，并安全实车标定；
    - 单调时钟看门狗、急停清缓存和恢复后新指令；
    - 状态门控（未知或不安全时不允许运动）；
    - `/odom` 回传（若 SDK 不给，从有效反馈积分）。
12. 先做无硬件回环，再按厂商认可的隔离/支撑方式验证动作、方向和硬急停。当前 step2 仅完成前者，不能据 smoke 推断腿/轮子的真实动作。

### 阶段 4：实车调试（1–2 天）
13. **先关跟随、只测急停**：手动推障碍物到雷达前 0.35 m 内，确认机器狗停住。这是**必须最先确认**的功能。
14. 用 `active=false` 起节点，先只看 `/rs_follow/target` / `/rs_follow/status` 输出是否正确，**不发速度**。
15. 再 `active=true` 低速试跟（把 `max_linear` 从默认 0.90 降到如 0.3），确认方向、量级、稳态距离。
16. **标定 `follow_dist` 与 `full_scale`**：让目标站定，测实际稳态距离，与期望值比对，反推修正。
17. 最后开放到正常速度。

---

## 6. 工作量与风险汇总

| 环节 | 工作量 | 风险 | 缓解 |
|---|---|---|---|
| ROS 2 环境 | Orin: 0.5 h / Xavier: 0.5–1 天 | Xavier 上 Humble 无官方 deb | 用 Docker（推荐）或源码编译 |
| 算法本体移植 | **0** | 无 | 无架构相关代码，无 PCL/OpenCV/Eigen |
| 雷达接入 | 0.5–1 天 | 驱动 arm64 编译；CustomMsg 格式 | 优先选有官方 arm64 支持的雷达 |
| `height_min/max` 标定 | 1 h | **高**：错了会全程急停 | 用卷尺量安装高度，按公式设 |
| TF / 外参 | 0–1 天 | 中：斜装雷达必须处理 | 优先正装；否则手写点云变换保零 PCL |
| 底盘桥接（A 类） | 配置及现场确认 | 控制入口/消息类型不匹配 | 使用现有 Twist/TwistStamped 适配器并核对话题 |
| 底盘桥接（B/C 类） | 现有适配器配置或新协议实现 | 型号权限、枚举、比例不明 | 优先复用 Unitree/M20 实现；其他协议仍需按接口开发 |
| `full_scale` 实车标定 | 半天 | 中：直接影响稳态误差 | README 已给卷尺标定法 |
| 急停验证 | 0.5 天 | **高（安全）** | 架空先验，实车再验；必须最先做 |

**总体判断**：
- **迁移到 Jetson**：**低风险、低成本**。算法纯 CPU、28 MB 内存、依赖极简、无 GPU 需求，Jetson 16 GB 的算力对它是严重过剩。唯一的真实变数是 Xavier NX 的 Ubuntu 20.04。
- **「通用所有机器人」**：架构解耦不等于所有型号即插即用。当前已有 `dog_adapters` 共用安全门控、Twist/TwistStamped 和官方 Unitree Sport 输出，以及 M20 basic_server 适配；其他协议仍需真实实现。每个新平台仍须确认控制接口、权限、状态、枚举和比例，验证物理急停/制动与雷达安装。统一 `follow_stack` 用显式 YAML 选择一个输出并保持 inactive 启动；已完成的无硬件 smoke 不能替代目标机与现场验证。

---

## 7. 附：可直接参考的仓库内文件

| 文件 | 作用 | 分支 |
|---|---|---|
| `src/rs_follow/README.md` | 话题表、参数、状态机、自带脚本 | main / algo-only |
| `src/rs_follow/config/follow_params.yaml` | 算法可调参数；适配器参数另见根 README | main / algo-only（历史分支） |
| `src/rs_follow/docs/PORTING.md` | 换雷达 / 换底盘 / TF 指引；输出章节已更新 | 当前工作树 |
| `src/rs_follow/launch/follow.launch.py` | 可同时拉起雷达驱动（`with_lidar` 参数） | main / algo-only（**两分支完全相同**） |
| `src/m20_bridge/README.md` | M20 协议表 + `fake_m20_server` 回环自测法 + `full_scale` 标定法 | **仅 main** |
| `src/m20_bridge/m20_bridge/bridge_node.py` | **B 类桥接的完整范例**（看门狗、量纲映射、状态门控、odom 回传） | **仅 main** |
| `src/m20_bridge/m20_bridge/protocol.py` | APDU 编解码 | **仅 main** |
| `src/dog_adapters/dog_adapters/` | 共用 CommandGate、Twist / TwistStamped、官方 Unitree Sport 适配 | 当前 step2 工作树 |

> 以下为此前审阅的**历史分支快照**，保留原始记录；不能用于描述加入 `dog_adapters` 后的当前工作树。
> **分支差异提醒（已逐条核验）**：`git diff --name-status main algo-only` 的输出**只有 9 个 `D`（deleted）条目，全部位于 `src/m20_bridge/`**：
> ```
> D  src/m20_bridge/README.md                      D  src/m20_bridge/package.xml
> D  src/m20_bridge/m20_bridge/__init__.py         D  src/m20_bridge/resource/m20_bridge
> D  src/m20_bridge/m20_bridge/bridge_node.py      D  src/m20_bridge/setup.cfg
> D  src/m20_bridge/m20_bridge/fake_m20_server.py  D  src/m20_bridge/setup.py
> D  src/m20_bridge/m20_bridge/protocol.py
> ```
> 合计 **9 files changed, 483 deletions(-)**，**没有任何一个非 `m20_bridge` 的文件被改动或删除**。因此：
> - `src/rs_follow/`（含 `docs/PORTING.md` 与 `launch/follow.launch.py`）**两个分支逐字节相同**——做通用化时 **algo-only 分支上同样有 `PORTING.md` 和 `follow.launch.py` 可看**；
> - 只有 `src/m20_bridge/`（M20 私有协议桥接范例）是 **main 独有**；
> - ⚠️ 一个**已知的文档内部不一致**：`src/rs_follow/README.md:232` 引用了 `docs/SLAM_FASTLIO2_PLAN.md`，但**该文件在 main 和 algo-only 上都不存在**（`git ls-tree -r --name-only {main,algo-only} -- src/rs_follow/docs` 均只返回 `PORTING.md`）。这是上游仓库自身的悬空文档链接，不影响算法功能。
> - 做**纯算法验证**时，用 **algo-only** 更干净（`REPORT.md` 的全部实测都在 algo-only 上完成）；做**桥接适配**时参考 main 的 `src/m20_bridge/`。

---

*本文的 Jetson 平台事实来自 NVIDIA 官方 JetPack Archive / JetPack 5.1 / JetPack 5.1.7 / JetPack 6.2 产品页与 ROS 2 Humble 官方安装文档；性能数据来自本机 WSL 实测（`scripts/bench_load.py`，原始输出见 `evidence/bench_load.txt`）；代码事实来自对 `main` 与 `algo-only` 分支源码的逐字阅读。*
