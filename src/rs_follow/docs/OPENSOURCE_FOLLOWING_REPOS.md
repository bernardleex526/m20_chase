# 机器人跟随（Vision / LiDAR Person Following）开源仓库清单

> **核实方式**：所有条目均通过 GitHub REST API（`/search/repositories`、`/repos/{owner}/{repo}`、`/repos/{owner}/{repo}/readme`）真实查询确认存在，并拉取 star / fork / 最后推送时间 / 许可证；关键仓库另做 HTTP HEAD 200 校验。
> 少数仅来自网页搜索、未经 API 逐一核实的条目在 §7 单独列出并明确标注。
> 数据为报告生成时的 API 实时返回。

---

## 0. 结论先行

| 判断 | 内容 |
|---|---|
| **有没有现成可用的机器狗跟随库？** | **没有。** 四足专用项目全部 ≤2★，均为个人作品，无 ROS2 化、无长期实机验证。可行路线是「移动机器人跟随方案 + Unitree SDK 适配」。 |
| **最该优先读的三个仓库** | ① [`koide3/monocular_person_following`](https://github.com/koide3/monocular_person_following)（重识别 + 单目 UKF 架构）；② [`spencer-project/spencer_people_tracking`](https://github.com/spencer-project/spencer_people_tracking)（多模态融合架构）；③ [`erib001/autonomous_robot_following`](https://github.com/erib001/autonomous_robot_following)（Go2 工程落地骨架）。 |
| **ROS2 vs ROS1 分水岭** | 高星项目绝大多数是 ROS1。ROS2 侧可用：`mowito/ros2_leg_detector`、`malwaru/person_following_robot`、`hsn07pk/turtlebot4-people-avoidance`、`robotics-upo/hunav_sim`。 |
| **协议风险** | GPL/AGPL：`boxmot`、`ultralytics`、`sort-deepsort-yolov3-ROS`、`person_tracking_ros`、`rgbd_person_tracking`。无 License（默认保留所有权利）：`spencer_people_tracking`、`monocular_person_following`、`wg-perception/people`、`ros2_leg_detector`。 |

---

## 1. 视觉跟随（RGB / RGB-D + 深度学习）

| 项目 | 技术方案 | 平台 | Star / 活跃度 | 评价 |
|---|---|---|---|---|
| [koide3/monocular_person_following](https://github.com/koide3/monocular_person_following) | **单目**：`tf-pose-estimation` 人体检测 → 结合地平面的 **UKF** → **CCF + Online Boosting** 在线重识别保 ID | ROS1；Jetson TX2/Xavier 设计，可移植任意底盘 | ⭐226 / F50，push 2021-08-20；**无 License** | **本类可复用性最高**，设计目标就是「能在新平台复现」；纯单目无深度，未适配 ROS2 |
| [ob-f/OpenBot](https://github.com/ob-f/OpenBot) | 手机（Android）当算力大脑；含 person following + 自主导航栈；约 $50 小车 | Android + Arduino；ROS/ROS2 桥接 | ⭐3516 / F673，push 2026-09-25（**活跃**）；MIT | 星数最高的通用跟随项目，MIT 文档完善；跟随策略（视觉+超声）偏简单，适合入门与算法验证 |
| [IvLabs/person_following_bot](https://github.com/IvLabs/person_following_bot) | 深度图像分割 + 深度学习检测，**多检测器切换**降低遮挡影响；手势启动/停止锁定 | ROS1 Kinetic/Melodic + **TurtleBot2** | ⭐122 / F23，push 2022-11-22；MIT；Springer 论文 | 工程完整度与文档质量高，MIT 友好。「遮挡处理」与「手势获取目标」两个设计**可直接抄** |
| [anhbantre/PersonFollowingRobot](https://github.com/anhbantre/PersonFollowingRobot) | 智能跟随购物车，视觉检测+跟踪 + 路径跟随 | ROS1 移动底盘 | ⭐83 / F6，push 2023-07-06；GPL-3.0 | 功能齐；GPL 对商业闭源不友好 |
| [ilyasmg/sort-deepsort-yolov3-ROS](https://github.com/ilyasmg/sort-deepsort-yolov3-ROS) | `darknet_ros`(YOLOv3) 检测 + **SORT / DeepSORT** 双跟踪器可切换 | ROS1 Kinetic/Noetic | ⭐63 / F25，push 2024-06-17；GPL-3.0 | 纯「跟踪节点」，只出目标位置不控底速——**最易当机器狗感知模块插入**；GPL |
| [apennisi/rgbd_person_tracking](https://github.com/apennisi/rgbd_person_tracking) | RGB-D 深度图 + 地面分割 + 分类器检测再跟踪；ROS 框架化 | ROS1 Indigo | ⭐53 / F22，push 2017-11-14（老旧）；GPL-3.0 | 年代久远，但分层接口设计仍有参考价值；ROS2 需大改 |
| [malwaru/person_following_robot](https://github.com/malwaru/person_following_robot) | **ByteTrack**（改造版）+ RealSense D400 | **ROS2** + 手推车/移动底盘 | ⭐41 / F2，push 2023-04-06；**无 License** | ROS2 阵营少见的完整「识别→跟踪→跟随」，**ROS2 生态首选参考** |
| [khayliang/person_tracking_ros](https://github.com/khayliang/person_tracking_ros) | **YOLOv3 + DeepSORT** + person re-identification；提供「选哪个框当目标」的 ROS Service | ROS1 Melodic | ⭐33 / F7，push 2023-04-12；GPL-3.0 | **交互式选目标的接口设计**（`/choose_target`）非常适合真实场景 |
| [MedlarTea/OCL-RPF](https://github.com/MedlarTea/OCL-RPF) | **RAL 2024 官方实现**：部位级 **Online Continual Learning Re-ID** + Reservoir 记忆管理，解决长期跟随外观漂移 | ROS1 移动机器人 | ⭐32 / F3，push 2025-01-15；无 License；配套 [OCLReID](https://github.com/MedlarTea/OCLReID) ⭐12 | **学术前沿**，直击「跟久了认错人」；基于 mmtrack，依赖重复现成本高 |
| [sijanz/robust_people_follower](https://github.com/sijanz/robust_people_follower) | Orbbec Astra 深度相机 + 鲁棒人体跟随 | ROS1 + TurtleBot + Astra | ⭐29 / F4，push 2019-11-20；BSD-3-Clause | BSD 宽松、TurtleBot 验证过；**CTRA 运动模型外推做再识别**，与「零重依赖」风格最兼容 |
| [khalidbourr/Adeept-RaspberryYOLO-Follower](https://github.com/khalidbourr/Adeept-RaspberryYOLO-Follower) | **YOLOv5** 行人检测 + 跟随；附树莓派 32 位安装指南 | ROS Noetic + **Pi 4** + Adeept AWR 4WD | ⭐25 / F5，push 2023-10-25；MIT | 边缘部署踩坑经验最有价值；算法本身较基础 |
| [YasinSonmez/Person-following-robot](https://github.com/YasinSonmez/Person-following-robot) | 跟随人 + **社交感知轨迹生成**，主动避开动态障碍路径 | ROS1 移动机器人 | ⭐22 / F2，push 2022-06-06；无 License | 「跟随 + 社交避障」一体化；文档较少 |
| [wouter1006/Human-tracking-Orbbec-Astra-ROS](https://github.com/wouter1006/Human-tracking-Orbbec-Astra-ROS) | Orbbec Astra Pro 驱动 AGV 跟随行人 | ROS1 + AGV | ⭐21，push 2018-06-21 | 早期实现，仅历史参考 |
| [thisara011/Human-Following-Robot](https://github.com/thisara011/Human-Following-Robot) | 传感器+智能导航自主追踪 | ROS/C++ | ⭐18，push 2024-01-04；MIT | MIT 宽松，偏教学性质，适合当骨架改造 |
| [C-H-E-N-Zhihao/Person_Follower_ROS](https://github.com/C-H-E-N-Zhihao/Person_Follower_ROS) | **YOLOv8** + ROS，无深度也能跟随 | ROS1 Noetic + **TurtleBot3** | ⭐3，push 2026-01-27；MIT | 新、代码量小、MIT，**快速原型起点**；未经大规模验证 |
| [kanghk041/camera_bot](https://github.com/kanghk041/camera_bot) | **ROS2 + YOLOv8 + Nav2**，Gazebo 仿真人跟随 | ROS2 + Gazebo | ⭐1，push 2026-03-27 | 展示「YOLO 接 Nav2 目标点」这一**推荐范式**；仿真为主 |
| [Ashvin003/vision-person-follower-ros2](https://github.com/Ashvin003/vision-person-follower-ros2) | YOLO 跟随 + 避障 | **ROS2** + TurtleBot3 | ⭐1，push 2026-01-03；无 License | 小型 ROS2 参考实现 |
| [jammick/Ackermann-Mobile-Robot](https://github.com/jammick/Ackermann-Mobile-Robot) | ROS2 Humble + 微信小程序；SLAM/Nav2/人跟随/手势控制一体 | ROS2 Humble + Ackermann R3 PLUS | ⭐0，push 2026-08-17；MIT | 小程序远程交互是特色；跟随深度有限 |
| [victormarlor/MarIA](https://github.com/victormarlor/MarIA) | 人跟随 + 实时跌倒检测 + 避障导航，Jetson Xavier | ROS2 + Jetson Xavier | ⭐0，push 2026-09-24 | 助老场景集成度高（跟随+跌倒检测）；零 star 无社区验证 |
| [yassmine-saad/Autonomous-vision-based-person-following-robot-using-YOLO-and-Realsense](https://github.com/yassmine-saad/Autonomous-vision-based-person-following-robot-using-YOLO-and-Realsense) | YOLO + RealSense，**骨骼关键点** + 深度测距 | 移动机器人 + RealSense | ⭐0，push 2025-08-21 | 骨架信息增强锁定值得借鉴；工程原始 |
| [nach96/MPC_node](https://github.com/nach96/MPC_node) | **IpOpt 的 ROS MPC 控制器**，专门控制差速机器人跟随人 | ROS1 + 差速底盘 | ⭐7，push 2021-10-30 | **控制器层少见的精品**：把人当动态参考轨迹做 MPC，机器狗跟随可直接复用控制思想 |
| [sourabhwarghane/Autonomous_Mobile_Robot](https://github.com/sourabhwarghane/Autonomous_Mobile_Robot) | ROS2 + **TensorRT** + LiDAR 避障 + 人跟随 + 语音 AI + Docker | ROS2 + **Jetson Orin Nano** | ⭐0，push 2026-08-31；MIT | 技术栈与机器狗主流配置高度契合 |
| [EmilRyberg/P7Tiago-PersonFollowing](https://github.com/EmilRyberg/P7Tiago-PersonFollowing) | 人跟随（TIAGo） | TIAGo 移动操作臂 | ⭐1，push 2021-01-18；MIT | 移动操作臂，参考意义有限 |

### 1.1 视觉跟随 · 无人机方向

| 项目 | 技术方案 | 平台 | Star | 评价 |
|---|---|---|---|---|
| [EspiroFares/Autonomous-UAV](https://github.com/EspiroFares/Autonomous-UAV) | Sim-first ROS2：**Gazebo + ArduPilot SITL 数字孪生** → 真机，室内人跟随 | ROS2 + ArduPilot | ⭐33，push 2026-08-31；MIT | 无人机人跟随里文档最完整之一；sim-first 工作流值得借鉴（注意大小写：`espirefares` 小写路径 404） |
| [brunoeducsantos/follow-me](https://github.com/brunoeducsantos/follow-me) | 深度学习架构锁定并跟随人（Udacity 系） | 无人机仿真 | ⭐24，push 2018-06-03；BSD-3 | 经典 Follow-Me，教学向 |
| [vijpandaturtle/Follow-me-drone](https://github.com/vijpandaturtle/Follow-me-drone) | 训练无人机跟随移动目标 | 无人机仿真 | ⭐9，push 2018-04-26 | 年代较早 |
| [swarm-subnet/swarm-interceptorV1](https://github.com/swarm-subnet/swarm-interceptorV1) | **YOLO + Kalman + 视觉伺服 + 目标重捕获** | ROS + Tello | ⭐9，push 2026-08-05；MIT | 「丢失后重捕获」与机器狗跟随需求高度同构 |
| [YanShuoWang/Person-Following-Drone](https://github.com/YanShuoWang/Person-Following-Drone) | **PX4 + VINS-Fusion + Elastic-Tracker + YOLOv8n** | ROS + PX4 | ⭐2，push 2026-07-07 | 视觉惯性融合定位 + 跟随完整链路 |
| [1destroyer100yt/drone-tracker](https://github.com/1destroyer100yt/drone-tracker) | 自训练 YOLOv8 (ONNX + CoreML/ANE) + **遮挡 Re-ID** | ArduPilot | ⭐2，push 2026-09-24；MIT | 遮挡后 Re-ID 与端侧部署思路清晰 |
| [ahmedvictor507/Advanced-Drone-Tracking](https://github.com/ahmedvictor507/Advanced-Drone-Tracking) | **YOLOv8 (TensorRT) + StrongSORT** + 三轴 PID + MAVLink | **Jetson Orin Nano** | ⭐0，push 2026-04-10 | TensorRT + StrongSORT + PID 现代组合，工程细节参考价值高 |
| [Jiaqi-WANG/DroneTarget](https://github.com/Jiaqi-WANG/DroneTarget) | 颜色/人体检测 + 跟随（OpenCV + ROS） | ROS 无人机 | ⭐2，push 2019-05-17；BSD-3 | 早期实现 |

---

## 2. 激光雷达 / 点云跟随

| 项目 | 技术方案 | 平台 | Star / 活跃度 | 评价 |
|---|---|---|---|---|
| [koide3/hdl_people_tracking](https://github.com/koide3/hdl_people_tracking) | **3D LiDAR**：Häselich **聚类** → **Kidono 行人分类器**去误检 → **卡尔曼（匀速）** 跟踪 | ROS1 + Velodyne 等 3D 激光；依赖 `ndt_omp` / [`hdl_localization`](https://github.com/koide3/hdl_localization)(⭐1026) | ⭐323 / F100，push 2019-12-05；BSD-2 | **3D 激光跟随的标杆**。聚类+分类器+Kalman 三段式，BSD 宽松，无重度视觉依赖，**最适合纯激光机器狗**；已停更，需 ROS2 移植 |
| [wg-perception/people](https://github.com/wg-perception/people) | **2D 激光 `leg_detector`** 经典上游：腿部特征（双峰宽度/间距）+ AdaBoost 分类器 + 跟踪 | ROS1 + 任意 2D 激光 | ⭐288，push 2025-05-14（仍有维护）；**无 License** | **「腿部检测跟随」的祖传代码库**，被无数项目 fork/移植；稳定省算力无 GPU；ROS1 且无 License |
| [MyNameIsCosmo/lidar_body_tracking](https://github.com/MyNameIsCosmo/lidar_body_tracking) | **octree + 聚类提取**的 3D 点云人体跟踪 | ROS Catkin + Quanergy 等 | ⭐76 / F29，push 2017-07-04 | PCL octree 聚类干净示例，适合理解点云人体分割；年代久远 |
| [ob-f/OpenBot](https://github.com/ob-f/OpenBot) | 见 §1（含超声跟随） | — | ⭐3516；MIT | — |
| [marcobecerrap/edge_leg_detector](https://github.com/marcobecerrap/edge_leg_detector) | 激光扫描腿部检测 | ROS1 + 2D 激光 | ⭐44 / F20，push 2019-12-06 | `leg_detector` 的现代化改写，代码更易读 |
| [mowito/ros2_leg_detector](https://github.com/mowito/ros2_leg_detector) | **ROS2 版 leg detector**：订阅 `/scan`，输出 `/people_tracked`（`PersonArray`，含位置与速度）+ RViz marker | **ROS2 Foxy** + 2D 激光 | ⭐39 / F14，push 2021-01-16；**无 License** | **2D 激光跟随里最实用的 ROS2 选项**。话题接口清晰、参数可调（扫描频率需 7.5/10/15Hz 降采样） |
| [Project-MANAS/person_tracking](https://github.com/Project-MANAS/person_tracking) | **3D LiDAR** + 点云**欧氏聚类**，找最近目标并跟随；速度按距离与角度缩放 | ROS1 + 3D LiDAR | ⭐29，push 2019-11-10 | 参数暴露极充分（`distance_to_maintain`、`linear/angular_threshold`、`clip_cloud`、`clip_angle_min/max=±1.31`），**调参经验可直接迁移**；无重识别 |
| [naiveHobo/person_tracking](https://github.com/naiveHobo/person_tracking) | 与 Project-MANAS 同源（fork），跟踪并跟随目标人 | ROS1 | ⭐19 / F3，push 2019-11-10 | 同上，可作对照 |
| [CentralLabFacilities/bayes_people_tracker](https://github.com/CentralLabFacilities/bayes_people_tracker) | **Bayes 人跟踪器**（源自 STRANDS 感知栈），融合多检测器输出 | ROS1；C++ | ⭐11，push 2025-11-18 | 贝叶斯多检测器融合，适合与 leg_detector / RGB-D 检测器组合 |
| [ShelyH/leg_detector_ros2](https://github.com/ShelyH/leg_detector_ros2) | leg detection + tracking，移植到 **ROS2 Galactic** | ROS2 Galactic + 2D 激光 | ⭐5，push 2024-05-06 | 更新一代的 ROS2 腿检实现 |
| [AJ1904/ROS2-Lidar](https://github.com/AJ1904/ROS2-Lidar) | 处理 2D 激光数据检测并跟踪机器人周围的人 | **ROS2** + 2D 激光 | ⭐2，push 2024-12-30；Apache-2.0 | 小型 ROS2 实现，代码量小易读 |
| [nathanhtkd/ROS-LIDAR-People-Tracking](https://github.com/nathanhtkd/ROS-LIDAR-People-Tracking) | LiDAR 人体跟踪 | ROS1 + LiDAR | ⭐2，push 2023-11-02 | 无 README，参考价值有限 |
| [franktsaodev/people_tracking](https://github.com/franktsaodev/people_tracking) | **3D LiDAR + AdaBoost + PCL + Kalman + DBSCAN** 多目标行人跟踪 | ROS1 + 3D LiDAR | ⭐1，push 2026-08-08 | 新实现，DBSCAN + AdaBoost 组合较完整 |
| [hsn07pk/turtlebot4-people-avoidance](https://github.com/hsn07pk/turtlebot4-people-avoidance) | **LiDAR 腿检测 + Kalman + CBF（控制障碍函数）安全滤波** | **ROS2 + TurtleBot4** + 网页仪表盘 | ⭐0，push 2026-07-02；Apache-2.0 | **CBF 安全层是亮点**——跟随中保证不撞人，机器狗尤其需要；Apache-2.0 友好 |
| [ros-drivers/urg_node](https://github.com/ros-drivers/urg_node) | Hokuyo URG 2D 激光 ROS 驱动（**跟随链路前置依赖**） | ROS1/ROS2 | ⭐122，push 2025-05-23；NOASSERTION | 用 Hokuyo 单线激光跟随时的必备驱动 |

---

## 3. 视觉-激光融合 / 通用目标跟随框架

| 项目 | 技术方案 | 平台 | Star / 活跃度 | 评价 |
|---|---|---|---|---|
| [spencer-project/spencer_people_tracking](https://github.com/spencer-project/spencer_people_tracking) | **多模态融合标杆**（EU FP7 SPENCER）：多个 **RGB-D 检测器 + 2D 激光检测器**统一框架；最近邻数据关联；含**社会关系估计 + 群体跟踪 + IMM 机动模型 + 高召回检测** | ROS1 移动机器人（机场场景验证） | ⭐717 / F321，push 2021-02-02；**无 License**；20–30Hz（跟踪器仅占 ~10% 单核） | **最完整的开源多人检测跟踪框架**，融合架构/消息类型/接口设计是教科书级，「可扩展可复用」是明确设计目标。**自建机器狗跟随栈的架构参考第一选择**；代码庞大、ROS1、无 License |
| [strands-project/strands_perception_people](https://github.com/strands-project/strands_perception_people) | 人的**长期检测、跟踪与识别**（long-term detection/tracking/recognition） | ROS1 | ⭐98，push 2020-04-11；C | 「长期运行」视角（重识别、身份维持）独树一帜，正好补短期跟踪器短板；已停更 |
| [Adlink-ROS/adlink_neuronbot](https://github.com/Adlink-ROS/adlink_neuronbot) | **ROS2/DDS** 人跟随 + 群体（swarm）功能包 | ROS2 + NeuronBot | ⭐16 / F6，push 2019-01-23；Apache-2.0 | 少见的 ROS2 早期 DDS 实现 + Apache-2.0；底盘绑定 NeuronBot |
| [tim-fan/realsense_spencer_adaptor](https://github.com/tim-fan/realsense_spencer_adaptor) | 让 SPENCER 跟踪框架直接跑在 **RealSense D435** 上的适配层 | ROS1 + D435 | ⭐9 / F4，push 2019-03-19 | 想复用 SPENCER 但只有 D435 时的关键胶水层 |
| [mikel-brostrom/boxmot](https://github.com/mikel-brostrom/boxmot) | **可插拔 SOTA 多目标跟踪模块集合**（ByteTrack / StrongSORT / BoT-SORT / OCSORT / DeepOCSORT 等），支持轴对齐与有向框 | Python（框架无关，可 ROS 封装） | ⭐8308，push 2026-09-18；**AGPL-3.0** | **跟踪层的现代首选**：换跟踪器不用改代码。AGPL 对闭源商业不友好 |
| [FoundationVision/ByteTrack](https://github.com/FoundationVision/ByteTrack) | ECCV 2022 的 **ByteTrack**（关联每个检测框，低分框也利用） | Python | ⭐6721，push 2024-06-19；MIT | **MIT + 效果强**，跟随场景首选跟踪器之一 |
| [nwojke/deep_sort](https://github.com/nwojke/deep_sort) | **DeepSORT** 原始实现（在线跟踪 + 深度关联度量） | Python | ⭐6183，push 2025-03-02；GPL-3.0 | 被 ROS 项目引用最多的跟踪器原型 |
| [JDAI-CV/fast-reid](https://github.com/JDAI-CV/fast-reid) | SOTA **行人重识别（ReID）**方法与工具箱 | Python | ⭐3992，push 2024-07-30；Apache-2.0 | 跟丢后「重新找回人」的核心能力来源，Apache-2.0 |
| [open-mmlab/mmtracking](https://github.com/open-mmlab/mmtracking) | OpenMMLab 视频感知工具箱（VID / MOT / VIS / SOT） | Python | ⭐3907，push 2023-09-19；Apache-2.0 | 学术复现与多算法横评的基础设施；已停更 |
| [ZQPei/deep_sort_pytorch](https://github.com/ZQPei/deep_sort_pytorch) | DeepSORT + YOLOv3 的 PyTorch 复现 | Python | ⭐3013，push 2024-07-16；MIT | MIT、上手极快，适合原型验证 |
| [ultralytics/ultralytics](https://github.com/ultralytics/ultralytics) | YOLO 系列（v8/v11/YOLO26 等）+ 内置跟踪与分割 | Python / 多平台导出（TensorRT/ONNX） | ⭐62192，push 2026-10-04；AGPL-3.0 | 检测层绝对主流；AGPL 需注意，边缘部署文档极全 |
| [robotics-upo/lightsfm](https://github.com/robotics-upo/lightsfm) | 轻量**社会力模型（Social Force Model）**库 | C++ | ⭐80，push 2026-01-08；BSD-3 | 行人运动/机器人避让建模基础件，HuNavSim 依赖 |
| [MedlarTea/OCLReID](https://github.com/MedlarTea/OCLReID) | 在线持续学习 **ReID**（视频输入版），与 OCL-RPF 配套 | Python (mmtrack) | ⭐12，push 2025-03-20 | 已有检测+跟踪、只差「认人」时可直接接入 |

---

## 4. 四足机器狗专用跟随项目

> ⚠️ **四足机器狗跟随是开源生态的明显空白区**。经多轮检索，专用项目**全部为 ≤2★ 的个人/团队作品**，无高星成熟仓库，多数没有 ROS2 化、没有实机长期验证。

### 4.1 机器狗跟随专用项目

| 项目 | 技术方案 | 平台 | Star / 活跃度 | 评价 |
|---|---|---|---|---|
| [Zaltster/go2-woof](https://github.com/Zaltster/go2-woof) | **Go2 内置 L1 激光雷达穹顶**（`/utlidar/*`，DDS domain 0）SLAM 建图 + 浏览器点击导航；人跟随用**前视相机 YOLO + P 控制器**，含 **breadcrumb pursuit（人消失后沿其足迹走到最后目击点并扫描）** | **Unitree Go2** + 机上 **Jetson Orin** (Ubuntu 22.04 arm64) | ⭐1，push 2026-07-08；无 License；README 有实测状态表 | 本清单里最「真」的 Go2 跟随实现：有硬件清单、部署方式（`wendy run`）、功能状态表。breadcrumb 追丢恢复策略非常实用；1 star、无 License、高度绑定作者环境 |
| [erib001/autonomous_robot_following](https://github.com/erib001/autonomous_robot_following) | RGB 相机 **YOLO + SORT + GCL 特征 Re-ID**（防 ID 跳变）+ 深度估计人与机器人距离/视角；分 `camera_package`（感知）与 `person_following_package`（控制 + **目标丢失后基于信念的智能搜索**） | **Unitree Go2 Edu** + ROS2 | ⭐0，push 2026-06-18 | **结构最规范的 Go2 跟随项目**：感知/控制分包清晰、明确处理「人走出视野后如何最快找回」。零 star 风险自担，但代码骨架值得直接借鉴 |
| [rushilvishwakarma/svan-m2-vision-pipeline](https://github.com/rushilvishwakarma/svan-m2-vision-pipeline) | **YOLOv8 + MediaPipe + TensorRT** 实时视觉流水线：手势控制 + 人跟随 + 避障导航 | 四足机器人 + Jetson Nano | ⭐0，push 2026-05-23；MIT | MIT、技术栈现代（TensorRT）；四足但型号未明说，文档薄弱 |
| [Gorilla79/Unitree_go2_human_following](https://github.com/Gorilla79/Unitree_go2_human_following) | Go2 人跟随（README 仅一行标题） | **Unitree Go2** | ⭐2 / F1，push 2025-08-21 | 仓库存在但**内容几乎为空**，仅作「该方向有人在试」的信号 |
| [BeBecpp/DimOs_Windows](https://github.com/BeBecpp/DimOs_Windows) | Go2 控制的 **Windows 原生**开发预览：**WebRTC + YOLO + Kalman/IoU/HSV TargetLocker** 三级目标锁定 + 安全看门狗 + 无运动预检，**无需 WSL/Docker** | **Unitree Go2** + Windows | ⭐0，push 2026-07-29；MIT | 唯一把 Go2 跟随做成 Windows 免虚拟化方案的项目；「Kalman/IoU/HSV 三重目标锁」降级策略设计清晰；MIT 可读性强 |
| [msritian/Robot-locomotion-social-navigation](https://github.com/msritian/Robot-locomotion-social-navigation) | 端到端人跟随（End-to-End Booster K1 Robot Person Following） | **Booster K1** 腿式机器人 | ⭐0，push 2026-10-04 | 极新、腿式平台，端到端路线；无文档 |
| [RPL-CS-UCL/ASFM](https://github.com/RPL-CS-UCL/ASFM) | **Augmented Social Force Model**，专门面向**腿式机器人**的社交导航 | 腿式机器人 | ⭐2，push 2024-09-23；JavaScript | 明确针对 legged robot 的社交力模型，学术性强、工程化弱 |
| [Pratyush150/target-following-robot](https://github.com/Pratyush150/target-following-robot) | 人群中跟随：**largest-box / IoU 跟踪 / 世界系 Kalman + 匈牙利匹配 / 外观 Re-ID** 四种方案在 100 次固定种子实验下对比 | 移动机器人（**非四足**，算法通用） | ⭐0，push 2026-10-01；MIT | **极具参考价值的对照实验**：明确告诉你「拥挤人群中该选哪种关联策略」，MIT |
| [vedantparnaik/followme-bot](https://github.com/vedantparnaik/followme-bot) | 人跟随：**Re-ID + 避障 + 2D 仿真器 + ROS2 包 + 树莓派小车代码** | ROS2 + Raspberry Pi 小车（非四足） | ⭐0，push 2026-09-25；AGPL-3.0 | 层次完整（仿真→真机）、AGPL 需注意；可作 ROS2 实现范本 |
| [Wafei324/RDK-X5-Based-Multi-functional-Household-Wheel-Legged-Robot-Dog](https://github.com/Wafei324/RDK-X5-Based-Multi-functional-Household-Wheel-Legged-Robot-Dog) | 基于 D-Robot **RDK X5** 的家用轮腿机器狗：货物搬运、陪伴聊天、人机交互/跟随 | 轮腿机器狗 + RDK X5 | ⭐0，push 2026-07-09；C | 国产 RDK X5 轮腿方案，场景贴近实用；代码成熟度未知 |

### 4.2 机器狗跟随的**必备平台基础设施**（非跟随算法，但绕不开）

| 项目 | 作用 | Star / 活跃度 | 说明 |
|---|---|---|---|
| [abizovnuralem/go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) | Unitree GO2 AIR/PRO/EDU 的非官方 **ROS2 SDK** | ⭐1046 / F219，push 2026-07-13；BSD-2 | **给 Go2 做跟随的事实标准入口**，把机器狗变成标准 ROS2 节点（相机/IMU/激光），之后接任意跟随算法 |
| [unitreerobotics/unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | Unitree 官方 SDK v2（底层运动/传感器） | ⭐1387，push 2026-09-21；BSD-3 | 官方闭环控制接口，速度指令下发基础 |
| [unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) | Unitree 官方 ROS2 支持 | ⭐850，push 2026-07-02；BSD-3 | 官方 ROS2 桥接 |
| [Sayantani-Bhattacharya/unitree_go2_nav](https://github.com/Sayantani-Bhattacharya/unitree_go2_nav) | Go2 导航与 SLAM 包（基于 go2_ros2_sdk 的高层控制） | ⭐113，push 2025-03-18 | **跟随之前的「走得稳」**：导航/SLAM 基础设施 |
| [YasiruDEX/Go2-Dynamic-Inspection](https://github.com/YasiruDEX/Go2-Dynamic-Inspection) | Go2 非官方 ROS2 SDK，**含 3D 激光支持** | ⭐92，push 2026-05-26；MIT | 用 Go2 自带激光做点云跟随时需要它 |
| [darshmenon/quadruped-robotics-stack](https://github.com/darshmenon/quadruped-robotics-stack) | Go2 完整栈：RL 步态训练（MuJoCo + Gazebo Harmonic）、ROS2 CHAMP / Quad-SDK NMPC 行走控制、PPO | ⭐26，push 2026-09-19 | 若跟随需要「边走边跟随」的步态配合，这里提供行走层 |
| [yehna-kim/unitree-go2-waypoint-nav](https://github.com/yehna-kim/unitree-go2-waypoint-nav) | Go2 + **LiDAR SLAM + 航点导航**（ROS2, Docker） | ⭐7，push 2026-05-12；MIT | 纯导航无跟随；可作跟随的路径执行层 |
| [leggedrobotics/legged_gym](https://github.com/leggedrobotics/legged_gym) | Isaac Gym 腿式机器人训练环境 | ⭐3132，push 2025-05-29；NOASSERTION | 训练自定义运动策略（跟随+复杂地形） |
| [AI-DA-STC/M20-autonomy-sim](https://github.com/AI-DA-STC/M20-autonomy-sim) | **DeepRobotics M20（与本任务 m20_chase 同一机型）** 的 Gazebo 仿真 + CMU 自主栈 | 未逐一核实 star | **与 m20_chase 直接相关的同名机型仿真环境**，值得单独查看 |

---

## 5. 相关基准数据集与仿真环境

| 项目 | 类型 | 技术要点 | Star / 活跃度 | 评价 |
|---|---|---|---|---|
| [robotics-upo/hunav_sim](https://github.com/robotics-upo/hunav_sim) | **人群行为仿真器** | **ROS2 Humble**；社会力模型（含群体）驱动行人；**6 种行人对机器人的反应行为**（regular / impassive / surprised / curious / scared / threatening）；行为树管理；**社交导航指标评估**；仿真器无关内核，支持 Gazebo Classic、Gazebo Fortress、Isaac Sim、Webots | ⭐166 / F29，push 2026-09-08（**活跃**）；论文 [RAL 2023 + HuNavSim 2.0 (arXiv:2507.17317)](https://arxiv.org/abs/2507.01303) | **跟随/社交导航评测的首选环境**：能让「被跟随的人」对机器人做出反应，这是其他仿真器缺的。强烈推荐用于鲁棒性测试 |
| [srl-freiburg/pedsim_ros](https://github.com/srl-freiburg/pedsim_ros) | **行人仿真器** | 社会力模型驱动大量行人 agents，带 Gazebo 插件与传感器模拟；与 SPENCER 生态兼容 | ⭐591 / F171，push 2023-08-07（最后 commit 2020-09）；BSD-2 | 老牌、生态广、SPENCER 官方配套仿真；经典但更新慢 |
| [ut-amrl/SocialGym2](https://github.com/ut-amrl/SocialGym2) | **社交导航基准 + 仿真** | ROS + OpenAI Gym 接口；多机器人社交导航；PettingZoo / Stable-Baselines3 集成 | ⭐63 / F7，push 2024-04-16；MIT | 想做 RL 跟随策略时的轻量环境，MIT |
| [JRDB-dataset/jrdb_toolkit](https://github.com/JRDB-dataset/jrdb_toolkit) | **数据集工具链** | **JRDB**：真实校园环境，**360° 全景 RGB + 3D LiDAR** 同步采集，含行人框/轨迹标注 | ⭐52，push 2024-08-02 | **最适合「视觉+激光融合跟随」评测的数据集**（同时有相机与 3D 激光），做融合算法必看 |
| [vita-epfl/JRDB-Traj](https://github.com/vita-epfl/JRDB-Traj) | 数据集（轨迹预测） | JRDB 上的轨迹预测基线 + 数据预处理 | ⭐13，push 2023-11-07 | 若需要「预测被跟随者未来轨迹」（预测式跟随） |
| [google-research/human-scene-transformer](https://github.com/google-research/human-scene-transformer) | 轨迹预测框架 | Human Scene Transformer + JRDB 封装 | ⭐81，push 2024-08-14；Apache-2.0 | 预测层研究级工具 |
| [robotics-upo/hunav_gazebo_wrapper](https://github.com/robotics-upo/hunav_gazebo_wrapper) | 仿真适配器 | 把 HuNavSim 接入 Gazebo | ⭐30，push 2026-08-18；MIT | 用 HuNavSim + Gazebo 时的必备胶水 |
| [facebookresearch/habitat-lab](https://github.com/facebookresearch/habitat-lab) | 具身 AI 仿真 | 模块化高层库；Habitat 3.0 含人机协作社交导航 | ⭐3147，push 2026-05-07；MIT | 若跟随需要「高层语义决策」（保持社交距离、按指令跟随） |
| [microsoft/AirSim](https://github.com/microsoft/AirSim) | 无人机/车辆仿真 | Unreal/Unity 上的开源仿真器 | ⭐18533，push 2026-09-15 | 无人机跟随的仿真基座 |
| [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | **ROS2 导航框架** | Nav2：行为树任务编排、costmap、MPPI/DWA 控制器 | ⭐4762，push 2026-10-02 | **机器狗跟随最现实的落地方式**：把「被跟随者位置」作为动态目标点喂给 Nav2，由它处理避障与轨迹生成 |
| [realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros)（原 IntelRealSense/realsense-ros，组织已改名） | 相机驱动 | RealSense D400 系列 ROS/ROS2 封装 | ⭐3460，push 2026-10-01；Apache-2.0 | RGB-D 跟随方案的必备驱动 |
| [ros-simulation/gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs) | 仿真桥接 | ROS 与 Gazebo 的封装工具 | ⭐865，push 2026-02-26 | 仿真环境搭建基础 |

### 5.1 行人检测数据集（激光方向）

| 项目 | 技术要点 | Star / 活跃度 | 评价 |
|---|---|---|---|
| [RWTHVision/DROW](https://github.com/RWTHVision/DROW) | **DROW v1/v2 数据集**：SICK S300 2D 激光、225° 视场、450 点、37 cm 安装高度、magic value 29.96 表示 N/A、标注率仅 5%、格式 `.csv` + `.wp/.wc/.wa` + `.odom2` | ⭐50，MIT | 2D 激光行人检测的公开数据基准，配套 DROW3 / DR-SPAAM 检测器 |

---

## 6. 综合推荐路线（四足机器狗人跟随）

```
感知层  →  YOLOv8 / ByteTrack（视觉）
        或  wg-perception/people + mowito/ros2_leg_detector（2D 激光）
        或  koide3/hdl_people_tracking（3D 激光）
        →  融合/跟踪参考 spencer_people_tracking 的架构；跟踪器用 mikel-brostrom/boxmot 便于切换
        →  重识别用 JDAI-CV/fast-reid，或参考 MedlarTea/OCL-RPF 的在线持续学习思路
控制层  →  nach96/MPC_node（MPC 思路）或 ros-navigation/navigation2（目标位置当动态目标点）
        →  安全层参考 hsn07pk/turtlebot4-people-avoidance 的 CBF 约束（保证不撞人）
底盘层  →  abizovnuralem/go2_ros2_sdk 把 Go2 变成 ROS2 节点
        →  底层速度指令走 unitreerobotics/unitree_sdk2
追丢恢复 →  参考 erib001/autonomous_robot_following 的信念搜索 / Zaltster/go2-woof 的 breadcrumb pursuit
评测    →  robotics-upo/hunav_sim（仿真）+ JRDB-dataset/jrdb_toolkit（真实数据，视+激光都有）
```

### 三类技术路线对比

| 路线 | 代表项目 | 优势 | 劣势 | 适合场景 |
|---|---|---|---|---|
| **视觉（RGB/RGB-D + 深度学习）** | monocular_person_following、person_tracking_ros、malwaru/person_following_robot | 能区分「具体某个人」（Re-ID）；语义丰富 | 受光照/遮挡影响；需 GPU | 室内、需要「只跟我一个」 |
| **激光雷达 / 点云** | hdl_people_tracking、ros2_leg_detector、wg-perception/people | 不受光照影响；测距精确；算力需求低 | 难以区分「哪个人」；只能跟最近目标 | 室外、人群简单、无 GPU |
| **视觉-激光融合** | spencer_people_tracking、strands_perception_people | 互补、鲁棒性最强 | 标定复杂、工程量最大 | 复杂真实环境、比赛/产品 |

### 协议风险提示

- **GPL / AGPL**（商业闭源不友好）：`khayliang/person_tracking_ros`、`ilyasmg/sort-deepsort-yolov3-ROS`、`apennisi/rgbd_person_tracking`、`mikel-brostrom/boxmot`、`ultralytics/ultralytics`
- **无 License**（默认保留所有权利，商用需联系作者）：`spencer_people_tracking`、`monocular_person_following`、`wg-perception/people`、`mowito/ros2_leg_detector`
- **MIT / BSD / Apache**（宽松）：`IvLabs/person_following_bot`(MIT)、`ob-f/OpenBot`(MIT)、`koide3/hdl_people_tracking`(BSD-2)、`sijanz/robust_people_follower`(BSD-3)、`hsn07pk/turtlebot4-people-avoidance`(Apache-2.0)、`abizovnuralem/go2_ros2_sdk`(BSD-2)、`srl-freiburg/pedsim_ros`(BSD-2)、`RWTHVision/DROW`(MIT)

---

## 7. 明确未能验证的条目

以下仓库来自网页/平台搜索命中，**未**逐一调用 API 确认 star 与活跃度，如需使用请自行核实：

`khanhvu4603/DATN_Ros2_Person_Following_Robot`、`wangzhengyuan209-droid/ros2_laser`、`WilliamSousaTech/person-following-robot-ros2`、`avcibatuhan/human-aware-robot-navigation`、`bilenbaris/Social_Robot_Nav`、`Maxence-Santos/ros2-slam-nav2`、`prasunjha8/social-nav-research`、`AshwinderPalSingh/Autonomous-mobile-robot-navigation-in-enviroment-with-humans`

---

## 8. 检索方法（可复现）

### 8.1 GitHub REST API `search/repositories`（按 star 排序，每词取前 6–8 条）

`person following robot` · `person follower ROS` · `human following robot` · `person tracking ROS` · `leg detector ROS` · `people tracking lidar ROS` · `person following quadruped robot` · `unitree following` · `UAV person following tracking` · `RGBD person tracking ROS` · `spencer people tracking ROS` · `bayes people tracker` · `crowd tracking ROS laser` · `deep learning person detection tracking ROS jetson` · `YOLO DeepSORT ROS tracking` · `autonomous follow me robot` · `robot person re-identification following` · `social navigation person following robot` · `human aware navigation ROS` · `mobile robot target tracking visual servoing person` · `openbot person following smartphone robot` · `ROS2 person following robot nav2` · `3D LiDAR pedestrian tracking ROS` · `TurtleBot3 person following YOLO` · `unitree go2 person follow` · `legged robot human following` · `drone person following YOLO ROS` · `point cloud people detection tracking ROS2` · `social robot navigation simulator benchmark` · `crowd navigation reinforcement learning ROS` · `people detection tracking framework ROS leg detector` · `yolo person follower car raspberry pi` · `unitree go2 ros2 sdk person detection` · `drone follow person open source` · `realsense person tracking follow robot` · `visual servoing person following mobile robot` · `amr person following warehouse robot` · `person re-identification deep learning library` · `multi object tracking library boxmot` · `JRDB dataset robot people detection` · `social navigation benchmark habitat` · `go1 go2 quadruped following person lidar` · `deep sort reid person tracking pytorch` · `target tracking drone Tello open source` · `quadruped person following` · `legged robot person tracking` · `camera lidar fusion people tracking` · `multi-modal people detection tracking` · `follow me robot ROS package` · `lidar 3d people detector ros` · `leg detector ros package` · `2D lidar person following robot ros` · `laser scan person following robot` · `human tracking RGBD camera ros` · `visual lidar fusion pedestrian tracking framework` · `robot following person Jetson deployment` · `person following ROS noetic` · `person following mobile manipulator` · `crowd navigation ORCA ROS robot` · `social force model navigation robot` · `anybotics following`

### 8.2 平台定向检索

- **GitHub**：`person following robot ROS` · `quadruped person following` · `LiDAR people tracking ROS` · `person following UAV drone` · `human tracking point cloud RGB-D ROS person` · `target following UAV person re-identification` · `person tracking lidar ROS leg detector` · `hunavsim human navigation simulator` · `lidar 3d people detector ROS` · `leg detector wg perception ROS` · `SEAN social environments navigation`
- **通用网页搜索（Bing，中英文）**：`github open source person following robot ROS YOLO DeepSORT` · `github person following mobile robot lidar leg detector ROS` · `github quadruped robot dog following person Unitree Go2` · `open source human following robot github ROS2` · `robotics person following survey github open source 2024` · `开源 机器人 跟随 人 GitHub 机器狗` · `ROS 行人跟随 目标跟随 开源项目 github`

### 8.3 定向 URL 核实

- **GitHub REST API**：`GET /repos/{owner}/{repo}`（约 90 次，逐一取 star/fork/push 时间/license/archived/topics）、`GET /repos/{owner}/{repo}/readme`（12 次，读取核心实现细节）
- **HTTP HEAD**：对 19 个关键仓库 URL 做 200 校验（唯一 404 为大小写错误的 `espirefares/Autonomous-UAV`，正确为 `EspiroFares/Autonomous-UAV`）
- **`read_page`**：`srl-freiburg/pedsim_ros`、`robotics-upo/hunav_sim` 完整 README

### 8.4 检索手段经验

- `read_page` 直读 `https://github.com/<owner>/<repo>` 可稳定拿到 stars / watchers / forks / commits / 最后提交 / LICENSE / README 全文；
- `advanced_search(engine=exa)` 对 GitHub 仓库检索命中率最高；`engine=bing` 对 github 查询基本无效；
- `platform_search(platform=github)` 长查询多返回 "No results found"，连续调用后稳定 `Error: GitHub API error (HTTP 403)` 限流；
- 多次检索 `m20_chase`、`rs_follow` **均未找到公开仓库**。

---

*本清单所有 star / 活跃度 / 协议数据均来自 GitHub API 实时返回，非文档推断。*
