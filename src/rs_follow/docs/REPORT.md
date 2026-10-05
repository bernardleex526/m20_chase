# m20_chase（algo-only）WSL Gazebo 实测验证报告

- 仓库：https://github.com/bernardleex526/m20_chase
- 分支/提交：`algo-only` @ `2cd0f6d`（feat(web): jie_deamon-style web console）
- 环境：WSL `Ubuntu-22.04` + ROS 2 Humble + **Gazebo Classic 11.10.2**（无 GPU，`gzserver` 无头）
- 被测量对象：**上游未修改**的 `rs_follow_node`（参数仅在命令行覆盖，源码零改动）
- 复现入口：`run_full_verification.sh`（主闭环）、`run_edge_verification.sh`（边界用例）
- 配套交付物：**GIF 可视化**（`evidence/gif/`，见 §2.7）、**Jetson 16GB 迁移与通用化适配评估**（`JETSON_MIGRATION.md`）、**同类开源跟随仓库调研**（`OPENSOURCE_FOLLOWING_REPOS.md`）

---

## 0. 结论速览

| 问题 | 结论 |
|---|---|
| 能否做「点云配准」？ | **能测准，但不是 ICP/NDT 意义上的配准**。算法把 3D 点云按高度带切成 2D 极坐标虚拟扫描（1440 bin / 0.25°），锁定的目标位置**对圆柱可见表面误差 n=9/9 平均 3.2 cm、最大 4.6 cm**；但仓库**完全不含**扫描匹配 / ICP / NDT / SLAM（见 §3）。 |
| 能否做「点云跟踪」？ | **能**。静目标稳态站位误差 **4.9 cm**；0.35 m/s 移动目标 mean standoff 误差 **5.7 cm**，机器狗位移 4.57 m 成功追上。 |
| 能否「机器狗跟随点云」闭环跑通？ | **能**。Gazebo 出真点云 → 节点算 `/cmd_vel` → 底盘真动 → 真实追上，4 个边界用例（急停/自动选点/丢失/安全闸）全部 PASS。 |
| 有没有致命问题？ | 有 2 条**安装/使用约束**必须满足：雷达必须**抬升到机体上方**（否则自遮挡触发急停）、`height_min` 必须按雷达实际离地高度标定（地面点占 97%）。 |
| 避障是否「完全满足工况」？ | **不能。** 实测为「**近身 0.35 m 环形急停 + 前向势场侧推**」，不是路径规划；正左/正右 0.30 m 即急停、**正后方全盲**、**被跟随者靠近到 0.45 m 会自己触发急停**、**无任何绕行能力**。详见 **§3.4**。 |

**附加交付**：六相位 GIF 动画（§2.7，真实遥测渲染）、Jetson 迁移评估（`JETSON_MIGRATION.md`，实测算法仅占 0.8–3.0 % 单核 / 28 MB）、开源跟随仓库调研（`OPENSOURCE_FOLLOWING_REPOS.md`）。

---

## 1. 测试方法

### 1.1 Gazebo 世界（自建 harness，`gazebo/follow_test.sdf`）

本机**只有 Gazebo Classic**（`/usr/bin/gzserver`；无 Ignition/Gazebo Sim，`/usr/share/gz` 不存在），
且非交互 shell 的 `DISPLAY` 为空，因此全部采用 `gzserver` 无头模式。

世界内三个模型：

| 模型 | 作用 | 关键参数 |
|---|---|---|
| `dog` | 机器狗替代体 | `base_link` 0.62×0.36×0.30 m；**雷达桅杆**抬高到机体上方（传感器在 world z=0.95，机体顶面 z=0.50）；`planar_move` 插件（全向，吃 `/cmd_vel`，发 `/odom`） |
| `target_person` | 被跟随的人 | 半径 0.25 m、高 1.7 m 圆柱，运行时用 `gz model -x -y -z` 传送 |
| `blocker_box` | 障碍物（急停用） | 0.2×1.0×1.5 m 方块，默认**停在 z=-100**（射线打不到），用时传送进场景 |

雷达：`libgazebo_ros_ray_sensor.so`，720 水平 × 16 垂直，±0.45 rad，0.20–30 m，10 Hz，高斯噪声 σ=0.01 m；
输出 `sensor_msgs/PointCloud2` → `/rslidar_points`，`frame_id=rslidar`。

> **雷达桅杆是必须的**。第一版把雷达直接放在机体几何中心，传感器看到自己的机身（~0.3 m），
> 直接触发算法自身的 `apf_emergency`（0.35 m）把 `/cmd_vel` 钉死为零。
> 这是**真实的安装约束**，不是 harness 缺陷：真机上雷达不能平齐或低于机体轮廓。

### 1.2 真值口径（关键修正）

第一轮测试报告了 20–28 cm 的「定位误差」，属于**真值口径错误**：把算法估计的**可见表面**位置
去比圆柱**轴心**。

- 激光只能看到圆柱的**朝向机器人那一侧的表面**，看不见轴心；
- 算法估计的正是表面（`radial` 恒 ≈0.21–0.23 m ≈ 圆柱半径 0.25 m）；
- 因此正确口径是 **est 到圆柱表面的距离**，而不是到轴心。

本报告采用修正后的口径，并同时给出两个数字（`err_surf` / `err_ctr`）。

### 1.3 被测量节点

```bash
ros2 run rs_follow rs_follow_node --ros-args \
  -p input_topic:=/rslidar_points -p active:=false -p odom_topic:=/odom \
  -p follow_dist:=1.0 -p height_min:=-0.60 -p height_max:=1.50 \
  -p cmd_vel_topic:=/cmd_vel          # 主闭环追加 -p auto_select_front:=true
```

`active:=false` 是仓库的安全默认值，测试中通过 `/rs_follow/enable` (Bool) 显式使能。

---

## 2. 实测结果

### 2.1 点云链路健康度（仓库自带 `pointcloud_status.py --duration 5`）

```
frames received : 51          rate : 10.19 Hz     frame_id : rslidar
fields          : ['x', 'y', 'z', 'intensity']
point_step/row  : 16 / 93888 bytes   dense=True
points/frame    : mean 5868         valid ratio : 100.0%
x range         : -25.02 .. +24.91  z range : -0.77 .. +0.96
horiz range     : p5 1.55  p50 2.75  p95 24.99
z histogram     : <-0.5: 97.0%   -0.5~0: 1.0%   0~0.5: 1.0%   >0.5: 0.0%
front cone pts  : mean 868 (|bearing|<30deg, range<5m)
```

> **97% 的点在 z<-0.5，也就是地面。** 这直接说明 `height_min` 是**第一优先级**参数：
> 不把地面滤掉，目标点簇会被地面点淹没。默认 `height_min=-0.40`，本测试按雷达实际
> 安装高度改为 `-0.60`（雷达在 z=0.95，地面在 z=-0.95 相对雷达）。

### 2.2 A) 点云测量精度 / 「配准」精度（9 个方位-距离组合）

| case | truth_rng | truth_brg° | est_x | est_y | err_surf | err_ctr | radial | pts | status |
|---|---|---|---|---|---|---|---|---|---|
| front_2m | 2.00 | 0 | 1.77 | -0.01 | 0.024 | 0.228 | 0.228 | 28 | TRACKING_MANUAL |
| front_3m | 3.00 | 0 | 2.79 | -0.01 | 0.040 | 0.213 | 0.213 | 20 | TRACKING_MANUAL |
| front_5m | 5.00 | 0 | 4.79 | -0.02 | 0.045 | 0.212 | 0.212 | 12 | TRACKING_MANUAL |
| right_3m | 3.00 | -90 | -0.01 | -2.78 | 0.026 | 0.225 | 0.225 | 19 | TRACKING_MANUAL |
| left_3m | 3.00 | 90 | 0.02 | 2.78 | 0.032 | 0.225 | 0.225 | 19 | TRACKING_MANUAL |
| front-right_3m | 3.00 | -45 | 1.96 | -1.97 | 0.027 | 0.223 | 0.223 | 19 | TRACKING_MANUAL |
| front-left_3m | 3.00 | 45 | 1.98 | 1.95 | 0.036 | 0.219 | 0.219 | 19 | TRACKING_MANUAL |
| diag_4m_30deg | 4.00 | 30 | 3.27 | 1.88 | 0.029 | 0.224 | 0.224 | 14 | TRACKING_MANUAL |
| back_3m | 3.00 | 180 | -2.78 | 0.01 | 0.028 | 0.225 | 0.225 | 20 | TRACKING_MANUAL |

```
error vs cylinder SURFACE     n=9/9  mean  3.2 cm   max  4.6 cm
error vs cylinder AXIS(centre) n=9/9  mean 22.2 cm   max 22.8 cm
radial (估计点到圆柱轴距离)   恒 ≈ 0.21–0.23 m  ≈ 圆柱半径 0.25 m
```

**解读**：
- 9/9 全部成功锁定，**无一例触发急停**，全向（含正后方 180°）都能测；
- 表面误差 2.4–4.6 cm，与雷达 σ=0.01 m 噪声 + 0.25° 分 bin 量化相当，接近理论上限；
- `radial` 恒等于圆柱半径 → **算法锁的是可见表面，不是轴心**，口径修正被数据证实；
- 方位精度：`front-left_3m` 与 `front-right_3m` 的 `est_y` 分别为 1.95 / -1.97（真值 ±2.12），
  误差来自表面 vs 轴心几何，不是角度偏斜。

### 2.3 B1) 静态目标跟随（起点 4 m，期望站位 1.0 m）

| t(s) | range(m) | dog_x(m) | vx | vy | wz | status |
|---|---|---|---|---|---|---|
| 0.3 | 3.767 | 0.022 | 0.422 | 0.000 | 0.000 | TRACKING_MANUAL |
| 2.3 | 2.669 | 1.149 | 0.900 | 0.000 | 0.000 | TRACKING_MANUAL |
| 4.3 | 1.303 | 2.472 | 0.900 | 0.000 | 0.000 | TRACKING_MANUAL |
| 6.4 | 0.858 | 2.908 | -0.080 | 0.000 | 0.000 | TRACKING_MANUAL |
| 8.4 | 0.880 | 2.889 | -0.118 | 0.000 | 0.000 | TRACKING_MANUAL |
| 11.1 | 0.907 | 2.861 | -0.129 | 0.000 | 0.000 | TRACKING_MANUAL |
| 13.2 | 0.940 | 2.832 | -0.107 | 0.000 | 0.000 | TRACKING_MANUAL |
| 15.2 | 0.951 | 2.819 | 0.000 | 0.000 | 0.000 | TRACKING_MANUAL |
| 17.2 | 0.952 | 2.819 | 0.000 | 0.000 | 0.000 | TRACKING_MANUAL |
| 19.2 | 0.951 | 2.819 | 0.000 | 0.000 | 0.000 | TRACKING_MANUAL |
| 21.2 | 0.952 | 2.819 | 0.000 | 0.000 | 0.000 | TRACKING_MANUAL |

```
稳态 standoff（末 7.8s）: 0.951 m  (目标 1.000 m, 误差 4.9 cm)
机器狗前进 2.80 m;  末态 vx=0.000 wz=0.000;  range 4.00 → 0.95 m
```

**PASS**。典型特征：先 0.9 m/s 满速接近，越过后轻微回退（-0.08 → -0.13 m/s）收敛，
15 s 后彻底静止（死区滞回起作用，无抖动）。

### 2.4 B2) 移动目标追遂（目标沿 +y 走 4 m，0.35 m/s）

| t(s) | range(m) | dog_x | dog_y | vx | vy | wz |
|---|---|---|---|---|---|---|
| 0.3 | 2.271 | 0.03 | -0.02 | 0.418 | 0.000 | 0.802 |
| 2.3 | 1.607 | 0.89 | 0.41 | 0.900 | 0.000 | 0.186 |
| 4.3 | 1.150 | 1.44 | 1.34 | 0.641 | 0.000 | 0.000 |
| 6.4 | 1.136 | 1.62 | 2.09 | 0.635 | 0.000 | 0.167 |
| 8.4 | 1.149 | 1.72 | 2.81 | 0.673 | 0.000 | 0.163 |
| 10.4 | 1.136 | 1.78 | 3.54 | 0.643 | 0.000 | 0.000 |
| 12.4 | 1.083 | 1.83 | 4.21 | 0.507 | 0.000 | 0.000 |
| 14.5 | 0.958 | 1.83 | 4.33 | 0.000 | 0.000 | 0.000 |
| 17.1 | 0.859 | 1.83 | 4.29 | -0.240 | 0.000 | 0.000 |
| 19.2 | 0.932 | 1.83 | 4.20 | -0.156 | 0.000 | 0.000 |

```
mean standoff（目标行走期间）: 1.057 m  (目标 1.000 m, 误差 5.7 cm)
机器狗从 (0.03,-0.02) 到 (1.83,4.19)  位移 4.57 m  → 成功追上
```

**PASS**。range 稳定在 1.08–1.15 m 跟随，机器狗横移 4.57 m 追平目标，末段目标停下后
自动收敛回 1.0 m 附近。

节点内部日志（节选，证明各子系统在工作）：
```
target=LOCK range=0.95 bearing=-0.1deg pts=48 min_obs=0.91 cmd=(0.00,0.00,0.00)
target=LOCK range=0.95 bearing=0.2deg pts=49 min_obs=0.90 cmd=(0.00,0.00,0.00)
slip comp: lin_ratio=1.00 ang_ratio=1.00 (actual/cmd)
bind target (2.00, 1.50) snapped=yes -> (1.75, 1.53)
```

`slip comp: lin_ratio=1.00` 说明 `planar_move` 不失真（理想底盘），打滑补偿链路已验证但本机无打滑可补。

### 2.5 C–F) 边界与安全用例（`run_edge_verification.sh`）

```
EDGE-CASE SUMMARY
  PASS  C obstacle emergency    障碍急停
  PASS  D auto target select    自动选目标
  PASS  E target loss           目标丢失
  PASS  F active gate           安全闸
```

**C) 障碍急停 PASS**
```
dog true pose x=2.82 y=0.00       (到位后)
blocker teleported to x=3.22 -> readback 3.21955 0.000117 0.75
blocker front face at x=3.12 (sensor at x=2.82), emergency band = 0.35 m
with obstacle : status=EMERGENCY_STOP  cmd=(0.00,0.00,0.00)  true_x=2.81
nonzero-cmd samples after obstacle: 0/216
```
障碍物前表面放在传感器前方 0.30 m（落在 `apf_emergency=0.35` 带内、又在
`frame_front=0.25` 自遮挡排除矩形之外），节点立即进入 `EMERGENCY_STOP` 并**瞬时归零不缓降**
（源码 `follow_controller.hpp:390` 在平滑器之前直接 return，属于安全设计）。

**D) 自动选目标 PASS**
```
status=TRACKING_AUTO  est=(1.2230, -0.0014)  est_range=1.22 m
cmd=(0.89,0.00,0.00)  nonzero-cmd samples while closing: 319/319
```
无任何手工 bind，`auto_select_front=true` 自行在正前方 ±40° 里锁定目标并驱车接近。

**E) 目标丢失 PASS**
```
before loss : status=TRACKING_MANUAL  cmd=(0.85,0.00,0.00)
coast window (~3 s, 允许非零): 105/162 nonzero samples
after loss  : status=NO_TARGET  cmd=(0.00,0.00,0.00)
nonzero-cmd samples after timeout: 0/256
```
目标移出世界后先**沿用最后已知位置惯性滑行约 3 s**（`lost_frames_timeout=30` @ 10 Hz，设计允许），
超时后彻底停发。对应源码 `follow_controller.hpp:357`：
```cpp
if (lost_frames_ > cfg_.lost_frames_timeout) { clearTarget(); res.target_valid=false; return res; }
```

**F) active 安全闸 PASS**
```
active=false, enable 未 latching: cmds seen 207, nonzero 0
```
`active:=false`（仓库默认）且未发 `/rs_follow/enable(true)` 时，**207 帧采样零非零指令**。

### 2.6 main vs algo-only 差异（回答「桥接是否有影响」）

```
git diff --stat main algo-only
  src/m20_bridge/  9 files, 483 deletions        ← 唯一差异
git diff --name-status main algo-only
  D src/m20_bridge/README.md
  D src/m20_bridge/m20_bridge/__init__.py
  D src/m20_bridge/m20_bridge/bridge_node.py
  D src/m20_bridge/m20_bridge/fake_m20_server.py
  D src/m20_bridge/m20_bridge/protocol.py
  D src/m20_bridge/package.xml
  D src/m20_bridge/resource/m20_bridge
  D src/m20_bridge/setup.cfg
  D src/m20_bridge/setup.py
git diff --stat main algo-only -- src/rs_follow
  (空输出)
```

**算法部分两分支逐字节相同**，`m20_bridge`（M20 真机速度接口桥）是纯增量。
因此本报告全部结论**对 main 分支同样成立**；用户「只用 algo-only 测」的授权不影响结论有效性。

> 逐条核验的补充结论（供迁移参考）：
> - `src/rs_follow/` 下**所有**文件（含 `docs/PORTING.md`、`launch/follow.launch.py`）**两分支完全相同**，
>   所以「换雷达/换底盘的官方指引」在 **algo-only 分支上同样存在**，不是 main 独有。
> - ⚠️ 上游自身的一个悬空链接：`src/rs_follow/README.md:232` 引用了 `docs/SLAM_FASTLIO2_PLAN.md`，
>   但该文件在 **main 和 algo-only 上都不存在**（`git ls-tree` 两分支的 `src/rs_follow/docs` 均只返回 `PORTING.md`）。

### 2.7 可视化：6 个相位的 GIF 动画（真实遥测渲染）

为让结果可直观看，把上面 A–F 六个用例各录成一段 GIF。**不是示意图**——每一帧都由
`gazebo_state` 插件的真值位姿 + 节点实际输出的话题数据渲染而成。

**录制链路**：`libgazebo_ros_state.so`（50 Hz 真值位姿，话题名是 **`/model_states`**）+
`/rs_follow/scan`（1440 bin 投影）+ 点云抽样 + `/rs_follow/target` + `/cmd_vel` + `/rs_follow/status`
→ `gz_record.py` 落 JSONL → `make_gifs.py` 渲染。

> `gz model -x -y -z` 每次调用约 **0.4–0.5 s**，从采样循环里直接调会把录制压到 0.6 Hz。
> 解法是独立 `Teleporter` 线程独享所有传送（`walk_line` 按墙钟时间重算位姿，因此目标
> 速度不受 CLI 延迟影响），录制循环只发布期望位姿。

| 相位 | 用例 | 样本 / 时长 | 关键观感 |
|---|---|---|---|
| `static.gif` | B1 静态目标接近与稳定 | 441 / 22.0 s | 狗从 4 m 处直冲，约 12 s 后停在目标前，`/cmd_vel` 归零 |
| `moving.gif` | B2 追逐步行目标 | 421 / 21.0 s | 目标沿 +y 匀速走 4 m，狗绕出弧线追随，最终航向 +75.9° |
| `accuracy.gif` | A 9 站测距 | 225 / 34.1 s | 目标被逐站传送 9 个方位-距离，狗全程静止只观测 |
| `obstacle.gif` | C 障碍急停 | 343 / 17.0 s | 盒子插入狗与目标之间 → 状态翻红 `EMERGENCY_STOP`，`/cmd_vel` 瞬时钉零 |
| `autoselect.gif` | D 自动选目标 | 241 / 12.0 s | 无任何手工绑定，狗自行在正前方锁定 6 m 外目标并驱近 |
| `loss.gif` | E 目标丢失 | 322 / 16.0 s | 目标被移出世界 → 估计 X 标记停驻原处、惯性滑行后翻 `NO_TARGET`、cmd 归零 |

**逐帧校验用的接触表**（`*_sheet.png`，各 6 帧拼图）确认：`accuracy` 的 9 站估计值稳定
偏离真值约 0.22 m（= 目标半径，即算法测的是**最近表面**而非圆心）；`obstacle` 在 14.17 s 起
cmd 全零且状态为 `EMERGENCY_STOP`；`loss` 在目标消失后估计标记停驻、12.63 s 起 `NO_TARGET`。
**各相位的行为与 §2.2–§2.5 的文本数据完全一致。**

交付文件：`m20_chase_verify/evidence/gif/` 下 6 个 GIF（3.59–8.63 MB，96 色量化）+
6 个接触表 PNG；量化指标原文见 `evidence/analyze_viz.txt`。

---

## 3. 能力边界（诚实说明）

### 3.1 仓库**有**的能力

| 能力 | 实现位置 | 说明 |
|---|---|---|
| 3D→2D 虚拟扫描投影 | `pointcloud_scan.hpp:64` `projectPointCloud()` | 高度带切片 + 方位分 bin 取最近，1440 bin（0.25°），雷达型号/线数无关 |
| 目标获取 | `follow_controller.hpp` | 自动选正前方 ±40°，或 `/clicked_point` / `/rs_follow/bind_target` 手工绑定（带表面吸附 `snapped=yes`） |
| 目标跟踪 | `follow_controller.hpp` + `kalman_filter_2d.hpp:237` | 目标半径内点簇质心 + 2D 卡尔曼（马氏门限 `kalman_gate=6.0`，R 按点数自适应），可切惯性系滤波 |
| 避障 | `follow_controller.hpp:290` | 人工势场排斥 + `apf_emergency=0.35` 急停 + `apf_slowdown=0.70` 减速 |
| 控制律 | `follow_controller.hpp` | 距离误差→`linear.x`，方位误差→`linear.z`，走廊→`linear.y`（全向底盘横移），含积分项抗移动目标滞后 |
| 输出平滑 | `cmd_smoother.hpp:103` | 加速度/加加速度限幅 + 一阶低通，急停例外 |
| 打滑补偿 | `rs_follow_node.cpp` | 按 `实际/指令` 比例放大速度（`slip comp`） |
| 失效保护 | `rs_follow_node.cpp` | `cmd_timeout=0.5` 无点云则缓降归零；`active`/`/rs_follow/enable` 双重安全闸 |

代码规模：`rs_follow` 共 **1492 行**（`follow_controller.hpp` 567，`rs_follow_node.cpp` 451，
`kalman_filter_2d.hpp` 237，`pointcloud_scan.hpp` 134，`cmd_smoother.hpp` 103），零 PCL 依赖。

### 3.2 仓库**没有**的能力（必须知道）

| 缺失 | 影响 |
|---|---|
| **ICP / NDT / 扫描匹配** | 无帧间配准，不做位姿估计；「配准」仅指投影后的目标位置测量 |
| **SLAM / 建图 / 定位** | 依赖外部 `/odom`（或退化为指令积分）；无地图 |
| **Re-ID / 外观识别** | 无法区分「是我的人」还是「别人」；目标丢失后只能重新自动选正前方最近目标，**不会主动找回** |
| **多目标跟踪 / ID 管理** | 同时只有 1 个被锁定目标 |
| **深度学习检测器** | 纯几何聚类，非行人分类器；会把正前方的柱子/墙也当目标 |
| 真机验证 | 无 M20 真机记录（`m20_bridge` 也未在本机验证） |

### 3.3 与同类开源项目的定位

能力边界与以下两个项目**同一档**：
- [`6-robot/jie_deamon`](https://github.com/6-robot/jie_deamon)（107★，m20_chase README 明确写「控制律参考」）——同样是卡尔曼 + 势场 + `/cmd_vel`；差别是它吃 **2D `LaserScan`**，m20_chase 吃 **3D `PointCloud2` 自研投影**（原创性更高）；
- [`naiveHobo/person_tracking`](https://github.com/naiveHobo/person_tracking)（19★）——3D 点云聚类 + 最近目标 + `/cmd_vel`，无避障。

**比 m20_chase 更完整**的参照：
- [`TeamSOBITS/sobits_follower`](https://github.com/TeamSOBITS/sobits_follower)（ROS 2 Jazzy，2D LiDAR DR-SPAAM + 云台 RGB-D SSD 双观测卡尔曼 + 虚拟弹簧跟随 + **DWA** 避障 + OSNet Re-ID）；
- [`gbhanuvigneshnaidu29052002-droid/Autonomous-Person-Following-Robot-with-Obstacle-Avoidance-Using-ROS-2`](https://github.com/gbhanuvigneshnaidu29052002-droid/Autonomous-Person-Following-Robot-with-Obstacle-Avoidance-Using-ROS-2)（同用势场，但多 YOLOv8+ByteTrack+**OSNet Re-ID** + 状态机 + 真机/Gazebo 双验证）。

**若只补最短板**：Re-ID 用 [`sijanz/robust_people_follower`](https://github.com/sijanz/robust_people_follower) 的 **CTRA 运动模型外推再识别**——几十行数学，**不引入深度学习依赖**，与 m20_chase「零 PCL、无重依赖」的风格最兼容。

### 3.4 避障能力边界实测（专项，回答「能否完全满足避障工况」）

**结论先行：不能。** 现有实现是「**近身环形急停 + 前向势场侧推**」，**不是路径规划意义上的避障**。它能在开阔场地对静态障碍**安全停下**，但**不能绕行**，且在窄道/人群中会频繁锁死，正后方是盲区。

#### 3.4.1 实现机制（源码事实，决定了能力上限）

| 环节 | 源码 | 行为 |
|---|---|---|
| 障碍距离 | `follow_controller.hpp:287-288` `if (!in_frame && dist < min_obstacle) min_obstacle = dist;` | 全向扫描取**最近一个标量**，**方向信息被丢弃** |
| 急停 | `follow_controller.hpp:390-393` `if (min_obstacle < cfg_.apf_emergency) { res.emergency_stop = true; res.cmd = cmd; return res; }` | `apf_emergency=0.35`；`cmd` 是默认构造的全 0 Twist，且在 `cmd_smoother` **之前** return → **瞬时硬归零，无缓降** |
| 减速 | `follow_controller.hpp:463-467` `f = clamp((min_obstacle - apf_emergency)/(apf_slowdown - apf_emergency), 0.1, 1.0); cmd.linear.x *= f;` | 只缩 `linear.x`，**不缩 `linear.y` / `angular.z`** |
| 势场 | `follow_controller.hpp:290-292` `force = apf_gain * (1/dist - 1/apf_influence)/(dist²)`，门限 `px > -0.1` | 仅对**前方**生效，`apf_influence=0.60` 之外无影响 |
| 自遮挡 | `follow_controller.hpp:283-285` `px > -frame_back && px < frame_front && py > -frame_right && py < frame_left` | 机体矩形内返回被判为自遮挡丢弃；`frame_front=0.25 frame_back=0.45 frame_left=0.25 frame_right=0.25` → **左右仅 ±0.25 m** |
| 投影 | `pointcloud_scan.hpp:113-122` `if (r < out.ranges[bin]) out.ranges[bin] = r;` | **每个 0.25° bin 只保留最近返回** → 同方位遮挡会顶掉目标 |

关键点：`min_obstacle` **未发布为话题**（只在 `rs_follow_node.cpp:238` 打日志），所以外部无法直接读取障碍距离，只能靠 `/cmd_vel` 与 `/rs_follow/status` 反推。

#### 3.4.2 实测方法

绕过 Gazebo，用 `scripts/obs_matrix2.py` / `obs_matrix3.py` 直接向 `/rslidar_points` 发**合成 `PointCloud2`**（0.15 m 半径人形圆柱簇 @3 m + 指定位置障碍簇，z 落在高度带内），保持高度带过滤与投影聚类路径被真实执行。每个用例前先 `/rs_follow/clear_target` 再重新 bind + enable（否则上一用例的急停会污染下一用例），每用例稳态测量 0.9 s。节点启动参数：`-p odom_topic:=/no_odom -p compensate_slip:=false -p height_min:=-0.60 -p height_max:=1.50 -p auto_select_front:=false`。

#### 3.4.3 实测结果（`evidence/obs_matrix2.txt`、`evidence/obs_matrix3.txt`）

**A. 急停包络是「环形气泡」而非前向锥**（目标恒在 3.0 m 正前方，障碍偏轴）

| 障碍位置 | 状态 | `vx` | `vy` |
|---|---|---|---|
| 无 | TRACKING_MANUAL | 0.896 | 0.000 |
| 正右 0.30 m (0,−0.30) | **EMERGENCY_STOP** | 0 | 0 |
| 正左 0.30 m (0,+0.30) | **EMERGENCY_STOP** | 0 | 0 |
| 正右 0.34 m | **EMERGENCY_STOP** | 0 | 0 |
| 正左 0.40 m | TRACKING_MANUAL | 0.469 | **−0.902** |
| 正右 0.40 m | TRACKING_MANUAL | 0.471 | **+0.903** |
| 右前 45° 0.30 m (0.21,−0.21) | **EMERGENCY_STOP** | 0 | 0 |
| 左前 45° 0.30 m (0.21,+0.21) | **EMERGENCY_STOP** | 0 | 0 |
| 贴身侧方 0.28 m (0.10,+0.28) | **EMERGENCY_STOP** | 0 | 0 |
| **正后方 0.30 / 0.40 / 0.60 / 1.50 m** | TRACKING_MANUAL | 0.900 | 0.000 |
| **后方斜 45° 0.297 m** (±) | TRACKING_MANUAL | 0.900 | 0.000 |

→ **正左/正右 0.30 m 就急停**（不只是前方），而**正后方 0.30 m 被完全无视**。急停区 = 以机体为中心、半径 0.35 m 的**环形气泡**（被自遮挡矩形挖掉左右各 0.25 m 的内核），**不是沿运动方向的路径锥**。

**B. 被跟随的人自己会触发急停**

| 目标距离 | 状态 | `vx` |
|---|---|---|
| 1.00 m | TRACKING_MANUAL | 0.000（正常站位） |
| 0.70 m | TRACKING_MANUAL | **−0.498**（后退） |
| 0.55 m | TRACKING_MANUAL | −0.562 |
| **0.45 m** | **EMERGENCY_STOP** | 0 |
| **0.40 m** | **EMERGENCY_STOP** | 0 |

→ 目标表面进入 0.35 m 时，**目标本身被当成障碍**。从 `follow_dist=1.0` 的安全余量只剩约 0.55 m，**人被挤到墙角/门框时必然锁死**。

**C. 无绕行能力**

全仓 grep `detour|bypass|replan|planner|astar|a_star|dwa|teb|rrt|costmap|occupanc` 在 `src/rs_follow/` 下 **零命中**（仅注释与 python 绘图里的 `grid`）。急停后**只能等障碍自己消失**，遇静态墙永久停车。

对称双障碍缝隙用例印证：0.6 m 间距缝隙 → 状态仍 TRACKING_MANUAL 但 **`vx` 塌到 0.014、`vy` 被推到 0.627**（严重侧偏），**不是穿缝而是被挤开**。

**D. 遮挡会使目标估计漂移，但不丢目标**

障碍放在视线上但气泡外（1.0–2.8 m）：

| 障碍距离 | 状态 | `est_rng`（基线 2.843） |
|---|---|---|
| 无遮挡 | TRACKING_MANUAL | 2.843 |
| 1.0 m | TRACKING_MANUAL | 2.988 (**+0.145**) |
| 1.5 m | TRACKING_MANUAL | 2.952 |
| 2.0 m | TRACKING_MANUAL | 2.931 |
| 2.5 m | TRACKING_MANUAL | 2.749 |
| 2.8 m | TRACKING_MANUAL | 2.876 |

→ 因「每 bin 只留最近」，遮挡确实顶掉目标的部分方位返回，估计被拉偏，**但偏差 ≤0.15 m 且不丢目标**（0.15 m 半径圆柱占多个 bin，尚有冗余）。**若目标更窄或距离更远，冗余会先耗尽。**

**E. 急停门槛精确标定（与参数一致）**

| 障碍距离 | 状态 | `vx` |
|---|---|---|
| 0.50 m | TRACKING_MANUAL | 0.899 |
| 0.45 m | TRACKING_MANUAL | **0.415**（减速生效） |
| 0.40 m | TRACKING_MANUAL | **0.041**（几乎停） |
| **0.37 m** | **EMERGENCY_STOP** | 0 |
| **0.35 m** | **EMERGENCY_STOP** | 0 |
| **0.33 m** | **EMERGENCY_STOP** | 0 |

→ 实测触发门槛 **0.35–0.37 m**，与 `apf_emergency=0.35` 吻合；减速带 0.40–0.70 m 生效。

#### 3.4.4 判定

| 工况 | 能否满足 | 依据 |
|---|---|---|
| 开阔场地、单个目标、前方静态障碍 → 安全停 | ✅ 可以 | 3.4.3-E |
| 前方障碍 → 轻微侧向绕开 | ⚠️ 部分（仅 0.40–0.60 m 带内产生侧推，且非规划） | 3.4.3-A |
| 窄走廊 / 双侧贴墙 | ❌ 频繁急停锁死 | 3.4.3-A |
| 人群穿行 / 行人从侧方经过 | ❌ 侧方 0.30 m 即急停 | 3.4.3-A |
| 需要绕行固定障碍（墙、柱、桌） | ❌ 无任何规划，永久停车 | 3.4.3-C |
| 正后方来车 / 后退时的障碍 | ❌ 完全盲区 | 3.4.3-A |
| 被跟随者贴近到 0.5 m 以内 | ❌ 目标自身触发急停 | 3.4.3-B |
| 目标被障碍遮挡 | ✅ 基本可以（漂移 ≤0.15 m） | 3.4.3-D |

**四点上真机前必须处理**：
1. `apf_emergency=0.35` 对**机体实际轮廓**可能过小（机身侧向伸出 >0.25 m 的实体不被自遮挡矩形覆盖时会被当障碍误停；反之真实宽度大于矩形时漏判近身侧碰）——需按真机外廓重新标定 `frame_*` 与 `apf_*`；
2. 后方 0.30 m 无任何保护 → 需补后向安全层（哪怕只是一个 `cmd.linear.x < 0` 时的后向距离门限）；
3. 目标自身触发急停 → 需把「被跟踪目标所在 bin」从障碍判定中**按目标 ID 排除**，否则站距小于 0.45 m 的工况不可用；
4. **无绕行** → 若场景存在固定障碍，必须外接 ROS 2 导航栈（`Nav2` costmap + 局部规划器）或至少实现「后退-侧移-再前进」的显式绕行状态机。

---

## 4. 上真机前的必做项

1. **雷达必须抬高到机体轮廓之上**。本测试第一版把雷达放在机体中心，传感器看到自身机身（~0.3 m）直接触发 `apf_emergency` 把 `/cmd_vel` 钉死为零。真机上雷达不能平齐或低于机体，或必须把机体自身点云通过 `frame_front/frame_back/frame_left/frame_right` 排除掉。
2. **标定 `height_min` / `height_max`**。地面点占 97%（见 §2.1），这两个参数按「雷达离地高度 + 行人高度范围」设定；本测试用 `-0.60 / 1.50`（雷达 z=0.95）。标错会直接导致目标点簇被地面淹没或目标躯干被滤掉。
3. **确认雷达朝向**。`flip_x` / `flip_y` 或 TF 修正；本测试假设 `frame_id=rslidar` 与机体同向（x 前 / y 左 / z 上）。
4. **首测保持 `active=false`**，先看 `/rs_follow/scan`（投影扫描）与 `/cmd_vel`，再逐步使能——本次测试验证了该安全闸确实有效（§2.5 F）。
5. **`enable_lateral`**：真机为全向底盘才设 `true`；差速底盘必须设 `false`，否则 `linear.y` 会被丢弃或造成异常。
6. 机器狗比轮式更怕顿挫，README 建议 `max_linear_accel: 0.5`、`max_angular_accel: 1.0`。
7. **避障四项（§3.4.4）**：把本体真实外廓标进 `frame_*`、补后向安全门限、把被跟踪目标的 bin 从障碍判定中排除（否则站距 <0.45 m 不可用）、若场景有固定障碍则必须外接 `Nav2` 或自写绕行状态机——**该仓库本身不会绕行**。

---

## 5. 复现方式

```bash
# 1. 克隆并切分支
git clone https://github.com/bernardleex526/m20_chase
cd m20_chase && git checkout algo-only
colcon build --symlink-install && source install/setup.bash

# 2. 主闭环（配准精度 + 静态/移动跟随）
bash run_full_verification.sh

# 3. 边界用例（急停 / 自动选点 / 丢失 / 安全闸）
bash run_edge_verification.sh

# 4. 避障包络矩阵（合成点云，绕过 Gazebo）
bash run_obs_matrix.sh    # GROUP 1+2 -> evidence/obs_matrix2.txt
bash run_obs_matrix3.sh   # GROUP 3+4+5 -> evidence/obs_matrix3.txt
```

证据文件：
- `m20_chase_verify/evidence/`：`A_scenario_test.txt`、`B_cloud_hz.txt`、`B_smoothness.txt`、`B_topic_list.txt`、`C_pointcloud_status.txt`、`D_gz_follow_test.txt`、`E_node_log.txt`、`F_gz_edge_test.txt`、`analyze_viz.txt`（六相位量化指标）、`obs_matrix.txt` / `obs_matrix2.txt` / `obs_matrix3.txt`（避障包络矩阵，§3.4）、`gzserver*.log`、`rs_follow_node*.log`、`obs_matrix*_node.log`
- `m20_chase_verify/evidence/gif/`：6 个相位 GIF + 6 个接触表 PNG（见 §2.7）

harness 文件：
- `m20_chase_verify/gazebo/follow_test.sdf`：Gazebo 世界（dog + 雷达桅杆 + target_person + blocker_box + `gazebo_state` 插件）
- `m20_chase_verify/scripts/gz_follow_test.py`：测量精度 rig
- `m20_chase_verify/scripts/gz_edge_test.py`：边界用例 rig
- `m20_chase_verify/scripts/gz_record.py` / `make_gifs.py` / `gif_frames.py` / `analyze_viz.py`：GIF 可视化链路（录制 → 渲染 → 抽帧校验 → 指标统计）
- `m20_chase_verify/scripts/run_full_verification.sh` / `run_edge_verification.sh` / `run_viz_record.sh`：编排脚本
- `m20_chase_verify/run2_output.txt` / `run_edge_output2.txt`：完整输出副本

**相关交付物**：Jetson 迁移与通用化适配评估见 `m20_chase_verify/JETSON_MIGRATION.md`；同类开源跟随仓库调研见 `m20_chase_verify/OPENSOURCE_FOLLOWING_REPOS.md`。

---

## 6. 环境坑位备忘（供后续复现）

| 坑 | 现象 | 正确做法 |
|---|---|---|
| `set -u` | `source /opt/ros/humble/setup.bash` 报 `AMENT_TRACE_SETUP_FILES: unbound variable` 直接退出 | **脚本顶部绝不能加 `set -u`** |
| CRLF | 脚本在 WSL 里报 `\r: command not found` | 复制前 `-replace "\`r\`n","\`n"` 或 `tr -d '\r'` |
| `pkill -f` 匹配自身 | `pkill -f gzserver` 杀掉自己的 shell | 写成 `pkill -f '[g]zserver'` |
| `gz model -s` | 不接受参数（读 stdin），`spawn_blocker` 传 SDF 静默失败 | 用**世界内定义 + `gz model -x -y -z` 传送**（本机唯一验证有效的 spawn 途径）；`gz model -f <file>` 返回 rc=0 但模型落在原点 |
| `gz model` 传送不重置 `/odom` | `planar_move` 的 odom 是航迹推算的，传送后与真实位姿发散，几何放置全错 | 用 `gz model -w <world> -m <model> -p` 读**真实世界位姿** |
| 无 GPU / DISPLAY 空 | RViz 与相机传感器不可用 | 一律 `gzserver` 无头 + 自建测量 rig |
| `platform_search(platform=github)` | 长查询返回 "No results found"，连续调用 `HTTP 403` 限流 | 短查询 + `read_page` 直读仓库页 |

---

*报告基于本机实测数据，非文档推断。所有数字均可在 `~/m20_chase/evidence/` 下复核。*
