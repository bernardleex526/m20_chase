# m20_chase / rs_follow 迁移 NVIDIA Jetson NX 16GB 与「通用机器人」适配评估

> 本文回答用户提出的第二个追加问题：
> 「m20_chase 的该仓库是否可以迁移至 nvidia nx 16gb 的上位机，然后适配不同机器人的 sdk 和桥接节点来达到通用所有的机器人」
>
> 结论先行：**可以迁移，而且几乎是「零改动迁移」**；「通用所有机器人」在**架构上是成立的**，但**不是靠一份代码跑遍所有机器人**，而是靠本文第 4 节的「三段式适配层」——算法内核不动，只在**输入投影**和**输出桥接**两端各写一个薄适配器。第 6 节给出工作量与优先级。

---

## 0. 结论速览

| 问题 | 结论 | 依据 |
|---|---|---|
| rs_follow 能否跑在 Jetson NX 16GB 上？ | **可以，CPU 余量极大** | 实测 131072 点 @10 Hz 时 **3.0 % 单核**、RSS **28 MB**（第 2 节） |
| 有没有 x86 专属代码需要重写？ | **没有，一行都不用改** | 全仓库无 SIMD/intrinsic/绝对路径（第 3.1 节） |
| 是否依赖 PCL / OpenCV / Eigen / CUDA？ | **全部不依赖** | 只依赖 6 个 ROS 消息包 + C++ 标准库（第 3.2 节） |
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

`src/rs_follow/CMakeLists.txt`（**main 与 algo-only 逐字节相同**，已用 `git diff` 核验）声明的依赖只有 6 个**纯消息/客户端包**：

```cmake
ament_cmake rclcpp sensor_msgs geometry_msgs nav_msgs visualization_msgs std_msgs
```

实际 `#include` 出来的全部头文件是：`<algorithm> <array> <chrono> <cmath> <limits> <memory> <string> <vector>`（C++ 标准库）+ rclcpp、geometry_msgs（Twist/PointStamped）、sensor_msgs（PointCloud2/LaserScan/PointCloud2Iterator）、nav_msgs（Odometry）、std_msgs（Bool/Int32/String）、visualization_msgs（Marker）。

**没有 PCL。没有 OpenCV。没有 Eigen。没有 CUDA。没有 Boost。** 编译产出一个单文件可执行 `rs_follow_node`（`add_executable(rs_follow_node src/rs_follow_node.cpp)`）。

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
| **A. 原生接受 `/cmd_vel`（Twist）** | 智元 D1 / jie_deamon、SW01（作者另一仓库）、绝大多数通用 ROS 底盘 | **零桥接**，只改 `cmd_vel_topic` 参数 | 分钟级 |
| **B. 私有 SDK / 私有协议** | **DeepRobotics M20**、部分宇树型号 | 写一个 Twist→私有协议的桥接节点 | 1–3 天 |
| **C. ROS 2 有官方包但消息类型非 Twist** | **Unitree Go2 / B2**（`unitree_api/msg/Request` Sport Mode 或 `unitree_go`） | 写 Twist→Unitree API 的桥接 | 0.5–1 天 |

**关于 M20（本仓库的 main 分支已经给出了一个现成范例）**：`src/m20_bridge/` 就是一个**完整的 B 类桥接实现**，可以直接当作「怎么写新桥接」的模板：

- **协议**（`m20_bridge/protocol.py`，64 行）：16 字节 APDU 头 + JSON ASDU。`SYNC = bytes([0xEB,0x91,0xEB,0x90])`，`FMT_JSON=0x01`；头部 = `SYNC + struct.pack('<H', len(body)) + struct.pack('<H', msg_id & 0xFFFF) + bytes([FMT_JSON]) + bytes(7)`。
- **指令**：`heartbeat()=encode(100,100)`、`set_mode(m)=encode(1101,5,{"Mode":m})`、`set_motion_state(p)=encode(2,22,{"MotionParam":p})`、`set_gait(g)=encode(2,23,{"GaitParam":g})`、`axis_cmd(x,y,yaw,...)=encode(2,21,{"X":...,"Y":...,"Z":...,"Roll":...,"Pitch":...,"Yaw":...})`。
- **桥接节点**（`bridge_node.py`，220 行）的**关键设计点，值得每个新桥接抄**：
  - **量纲归一化**：M20 的轴指令是 `[-1,1]` 的**最大速度比例**，不是 m/s。所以桥接里有 `full_scale_x/full_scale_y/full_scale_yaw`（默认 2.0 / 1.0 / 1.5），做 `x = clamp(vx/full_scale_x)`。**这就是「通用化」必须处理的第一件事：把算法输出的物理量纲，映射到目标平台的抽象量纲。**
  - **看门狗**（安全关键）：`stale = (time.time()-self.last_cmd_t) > self.watchdog_timeout`，默认 0.5 s；一旦超时就发全 0。**这是任何桥接都必须有的**——因为算法节点崩溃或网络断了，机器人必须停。
  - **状态门控**：`safe = stale or self.hes or (self.mode_now not in (-1,0))`——机器人不在「常规模式」时不接受速度指令。
  - **状态回传**：订阅 Type1002/Command6（`BasicStatus.HES/ControlUsageMode/MotionState/Gait`）与 Type1002/Command4（`MotionStatus.Yaw/LinearX/LinearY/OmegaZ`），积分成 `nav_msgs/Odometry` 发布到 `/m20/odom`。**注意**：算法需要 `/odom`（`follow_params.yaml` 的 `odom_topic=/odom`）来实现 `compensate_slip`（滑移补偿）和 `filter_in_world`（世界系滤波）。很多厂商 SDK 不直接给 odom，需要自己从轴反馈积分——M20 桥接的做法就是范例。

> **通用化的真正难点，在 `full_scale` 的标定**：M20 的 README 明确给了标定法——「卷尺测实际速度 v，`full_scale_x = v / 轴值`」。**每一个新机器人上车，第一件事都是标定这三个数**，否则跟随的稳态误差会直接体现为「站得太近/太远」。`REPORT.md` §2.3/§2.4 实测到的 **4.9 cm（静态）/ 5.7 cm（移动）** 稳态站位误差里，就包含这类标定误差与仿真底盘建模误差。

> **两条上游文档的悬空引用（已逐条核验，供参考时注意）**：
> 1. `docs/PORTING.md` 第三节写「现成参考：`sw01_dog_follower/sw01_dog_follower/robot_bridge.py`」，
>    并提到「本项目 SW01」。但 **`sw01_dog_follower` 在本仓库中不存在**（`find . -iname '*sw01*'` 为空，
>    全仓唯一的 `package.xml` 是 `src/rs_follow/package.xml`）——它是作者**另一个仓库**里的包。
>    所以「SW01 零桥接直接可用」这条结论的**代码范例在本仓库里拿不到**。
> 2. `src/rs_follow/README.md:232` 引用 `docs/SLAM_FASTLIO2_PLAN.md`，该文件**两个分支都不存在**。
>
> 这两处都是上游文档问题，不影响算法功能，但会影响「照文档走」时的预期。
> **真正可用的桥接范例只有 main 分支的 `src/m20_bridge/`。**

### 4.3 第 1 段（输入投影）：换雷达的坑比换底盘更深

`docs/PORTING.md` 把这一节列为第一优先级，而且反复强调 **`height_min` 是第一优先级参数**，理由是逐字的：

> `height_min ≈ -(安装高度) + 余量`，`height_max ≈ 行人高度 - 安装高度`，否则俯视地面的雷达会把地面当障碍触发 `apf_emergency`。

**这是整份文档里最容易被忽略、但最容易导致「一上车就急停」的坑。** 本次 WSL 测试也独立验证了它的重要性：`follow_test.sdf` 里给狗加了**雷达桅杆**把雷达抬高到机体轮廓之上，并且**把默认的 `height_min=-0.40 / height_max=1.80` 覆盖为 `-0.60 / 1.50`**（三个编排脚本 `run_full_verification.sh`、`run_edge_verification.sh`、`run_viz_record.sh` 均如此），正是为了让高度带匹配雷达实际安装高度（z=0.95 m）与行人躯干范围（见 `REPORT.md` 第 1.1 节）。

换雷达的具体做法：

| 雷达 | 话题 | 做法 |
|---|---|---|
| RoboSense（当前） | `/rslidar_points` | 默认，`input_topic` 即可 |
| Livox Mid-360 | `/livox/lidar` | 需 `xfer_format: 1` 输出 PointCloud2 |
| Velodyne | `/velodyne_points` | 直接改 `input_topic` |
| Ouster | `/ouster/points` | 直接改 `input_topic` |
| Hesai | `/hesai/pandar` | 直接改 `input_topic` |

**坐标系是另一个必做的坑**：`docs/PORTING.md` 明确指出，当前算法**隐含假设「点云坐标系即机体坐标系，且 +x 朝前」**，而 `flip_x` / `flip_y` 两个参数**只能处理轴翻转，处理不了安装位置偏移和安装旋转**。正确做法是引入 TF：

> 加 `base_frame` / `use_tf` 参数 + `tf2_ros::Buffer` + `tf2::doTransform`，并增加 `tf2_ros` / `tf2_sensor_msgs` / `pcl_ros` 依赖。

**这里有一个重要提醒**：这个改造会**打破「零 PCL 依赖」的现状**（`tf2_sensor_msgs` 用 PCL 做点云变换）。如果雷达可以做到「正装、+x 朝前」，建议先用参数凑合、保住零 PCL；如果雷达到底是斜装的，那就接受引入 PCL，或者自己手写一个 `PointCloud2 → 变换 → PointCloud2` 的变换（用 `sensor_msgs::PointCloud2Iterator` 逐点乘 4×4 矩阵，约 50 行，无 PCL 依赖）。**本项目作为一个小型算法项目，我推荐后者。**

---

## 5. 迁移执行清单（可直接照着做）

### 阶段 1：环境（半天）
1. 确认模块是 **Xavier NX 还是 Orin NX**（第 1 节，这决定后续路径）。
2. 装 ROS 2 Humble：Orin NX → `apt`；Xavier NX → Docker 或源码编译。
3. `git clone` + 切 `algo-only` 分支（本次验证用的分支，**排除了 M20 私有协议，纯算法**）。
4. `colcon build`。**预期零依赖报错**——因为只需那 6 个消息包。

### 阶段 2：雷达接入（1 天）
5. 装雷达驱动（RoboSense → `rslidar_sdk`；Livox → `livox_ros_driver2`）。
6. **先只验证传感器**：`ros2 topic hz /rslidar_points` 看频率，跑 `pointcloud_status.py` 看点云健康度。
7. **标定 `height_min` / `height_max`**：用卷尺量雷达离地高度 `h`。设 `height_min = -h + 0.2`（留余量），`height_max = 1.8 - h`。**这一步没做对，后面全是急停。**
8. 如果雷达斜装 → 按第 4.3 节加变换。

### 阶段 3：桥接接入（1–3 天，取决于机器人类型）
9. 判断目标机器人属于 A/B/C 哪一类（第 4.2 节）。
10. A 类：只改 `cmd_vel_topic`，收工。
11. B/C 类：**以 `m20_bridge` 为模板写新桥接**，务必包含：
    - `full_scale_x/y/yaw` 量纲映射（并实车标定）；
    - **看门狗超时归零**（安全底线，不可省）；
    - 状态门控（机器人不处于可接受指令的状态时不发速度）；
    - `/odom` 回传（若 SDK 不给，从轴反馈积分）。
12. 在**架空/吊装**状态下先验证桥接：发 `cmd_vel`，看轮子/腿是否按预期动作，方向对不对（**符号错了会直接冲向人或退向墙**）。

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
| 底盘桥接（A 类） | 分钟级 | 低 | 只改参数 |
| 底盘桥接（B/C 类） | 1–3 天 | 中：私有协议需逆向/查文档 | 照抄 `m20_bridge` 的四要素 |
| `full_scale` 实车标定 | 半天 | 中：直接影响稳态误差 | README 已给卷尺标定法 |
| 急停验证 | 0.5 天 | **高（安全）** | 架空先验，实车再验；必须最先做 |

**总体判断**：
- **迁移到 Jetson**：**低风险、低成本**。算法纯 CPU、28 MB 内存、依赖极简、无 GPU 需求，Jetson 16 GB 的算力对它是严重过剩。唯一的真实变数是 Xavier NX 的 Ubuntu 20.04。
- **「通用所有机器人」**：**架构方向正确，但不是免费的**。它是一个**每接一款新机器人需要 1–3 天适配**的方案，而**不是**一个「装上就通吃」的方案。真正需要投入的，是把第 4 节的「三段式适配层」**固化成接口约定 + 桥接模板 + 标定流程**，让每接一款新机器人的成本从「重新理解整个系统」降到「填三个 `full_scale` 数字、量一次雷达高度」。**`m20_bridge` 已经就是这个模板的第一个实例——把它抽成通用骨架，是这个仓库走向「通用」最值得做的一步。**

---

## 7. 附：可直接参考的仓库内文件

| 文件 | 作用 | 分支 |
|---|---|---|
| `src/rs_follow/README.md` | 话题表、参数、状态机、自带脚本 | main / algo-only |
| `src/rs_follow/config/follow_params.yaml` | **全部可调参数**（86 行），迁移时的首选调整入口 | main / algo-only |
| `src/rs_follow/docs/PORTING.md` | **换雷达 / 换底盘 / TF 的官方指引**（7918 字节） | main / algo-only（**两分支完全相同**） |
| `src/rs_follow/launch/follow.launch.py` | 可同时拉起雷达驱动（`with_lidar` 参数） | main / algo-only（**两分支完全相同**） |
| `src/m20_bridge/README.md` | M20 协议表 + `fake_m20_server` 回环自测法 + `full_scale` 标定法 | **仅 main** |
| `src/m20_bridge/m20_bridge/bridge_node.py` | **B 类桥接的完整范例**（看门狗、量纲映射、状态门控、odom 回传） | **仅 main** |
| `src/m20_bridge/m20_bridge/protocol.py` | APDU 编解码 | **仅 main** |

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
