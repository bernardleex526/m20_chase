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
height_min: -0.10               # ← 重点：Mid-360 垂直FOV -7°~+52°，地面回波多
height_max: 1.80
flip_x: false                   # Mid-360 默认 x前 y左 z上，与 rs 一致
flip_y: false
```

**高度带是第一优先级**。Mid-360 俯视地面，若 `height_min` 太低会把地面当作障碍物
（触发 `apf_emergency` 急停），必须按雷达安装高度重标：
`height_min ≈ -(安装高度) + 余量`，`height_max ≈ 行人高度 - 安装高度`。

### 3. 可能需要的小改动

| 情况 | 改动位置 | 做法 |
|------|----------|------|
| 消息不是 XYZ PointCloud2 | `pointcloud_scan.hpp` | 加消息类型分支 / 转换节点 |
| 坐标轴朝向不同 | 参数 | `flip_x/flip_y`；安装有旋转角则建议上 TF（见第二节）|
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

现在算法**隐含假设**：点云坐标系就是机体坐标系，且 `+x` 朝前。
通过 `flip_x/flip_y` 只能处理轴翻转，处理不了**安装旋转角/平移**。

正确做法（建议下一步加）：

1. 用 URDF/静态 TF 描述 `base_link → lidar`（含安装高度、俯仰、偏航）。
2. 在节点里加 `tf2_ros::Buffer`，把每帧点云/绑定目标变换到 `base_link` 再算方位距离。
3. 对应改动：
   - `pointcloud_scan.hpp`：先用 `tf2::doTransform` 变换点（或用 `pcl_ros`/`tf2_sensor_msgs`）。
   - `rs_follow_node.cpp`：新增参数 `base_frame`、`use_tf`，绑定点击点时也做变换。
   - `CMakeLists.txt` / `package.xml`：加 `tf2_ros`、`tf2_sensor_msgs`(可选)、`pcl_ros`(可选)。

雷达**安装高度变化**后必须重标：
- `height_min/height_max`（地面/行人相对高度）
- `follow_dist`（期望距离）
- `frame_front/back/left/right`（自遮挡盒）

---

## 三、更换机器狗

### 1. 速度接口

| 底盘 | 接口 | 需要做什么 |
|------|------|-----------|
| 通用 ROS 底盘 | `/cmd_vel` (`geometry_msgs/Twist`) | 只改 `cmd_vel_topic` |
| Unitree Go2/B2 | `unitree_api/msg/Request`（Sport Mode）或 `unitree_go` | **写一个 Twist→Unitree API 桥接节点** |
| 智元 D1 / 本仓库 jie_deamon | `/cmd_vel` + `/d1_cmd` | 直接可用 `/cmd_vel` |
| 本项目 SW01 | `hypertron-sw01` 驱动 `/cmd_vel` | 直接可用 |
| 自定义串口狗 | 私有协议 | 写桥接节点 |

桥接节点是最常见的适配工作，形态：

```
/cmd_vel (Twist) ──► [dog_bridge] ──► /your_dog/cmd  (私有协议/API)
```

现成参考：`sw01_dog_follower/sw01_dog_follower/robot_bridge.py`。

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
| 雷达安装高度/角度 | `height_min/max`、`z_offset`、必要时 `flip_*`/TF |
| 狗体尺寸（自遮挡） | `frame_front/back/left/right` |
| 期望跟随距离 | `follow_dist` |
| 目标大小（人/物） | `target_radius` |

### 4. 安全（换平台务必检查）

- 保持 `active: false` 默认，先看 `/rs_follow/scan`、`/cmd_vel` 再开启。
- 保留 `cmd_timeout` 失联保护与 `apf_emergency` 急停。
- 在桥接节点里再加一层速度限幅/看门狗，双保险。

---

## 四、改动清单总结

| 目标 | 只改配置 | 需要改代码 |
|------|:--------:|:----------:|
| 换同为 XYZ PointCloud2 的雷达 | ✅ `input_topic`+`height_*`+`range_*` | — |
| 换消息格式特殊（Livox CustomMsg / LaserScan） | — | `pointcloud_scan.hpp` 或加转换节点 |
| 雷达有安装旋转/平移 | 简单翻转可 | 建议加 TF (`tf2`) |
| 换 `Twist` 接口的狗 | ✅ `cmd_vel_topic`+速度/运动学参数 | — |
| 换非 `Twist` 接口的狗 | 参数 | 加 `dog_bridge` 桥接节点 |
| 换狗尺寸/雷达高度 | ✅ `frame_*`、`height_*`、`follow_dist` | — |

**核心原则**：控制律 (`follow_controller.hpp`) 与传感器、底盘解耦；适配工作集中在
「输入投影」和「输出桥接」两端。
