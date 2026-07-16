# DjiPayloadSDKInterfaceForROS1

> 基于 [DJI Payload SDK (PSDK)](https://github.com/dji-sdk/Payload-SDK) 的 ROS1 (Noetic) 集成开发平台，为 DJI 无人机提供传感器数据订阅、飞行控制和设备驱动等完整接口。
>
> 本项目引用 DJI Payload SDK V3.x，对应的 DJI 官方文档：[PSDK 开发教程](https://developer.dji.com/doc/payload-sdk-tutorial/cn/) | [PSDK API 参考](https://developer.dji.com/doc/payload-sdk-api-reference/cn/)

---

## 1. 项目简介

DjiPayloadSDKInterfaceForROS1 是一个 ROS1 catkin 工作空间，封装了 DJI Payload SDK 的核心功能，旨在帮助开发者**无需额外配置编译 DJI PSDK 即可在 ROS 环境下快速开发无人机应用**。

项目由以下功能包组成：

| 功能包 | 说明 |
|--------|------|
| `payload_sdk_ros1` | **核心** DJI PSDK 接口节点，实现飞控数据订阅、飞行控制、感知可视化 |
| `custom_msgs` | 自定义 ROS 消息与设备驱动（飞控协议、Livox 激光雷达、JP 激光/毫米波雷达） |

---

## 2. 项目架构

```
┌──────────────────────────────────────────────────────────────┐
│                      DJI 无人机                               │
│  (X-Port / SkyPort / 扩展接口 → PSDK 协议, UART/USB/网口)      │
└─────────────────────┬────────────────────────────────────────┘
                      │ DJI PSDK Library (libpayloadsdk.a)
                      ▼
┌──────────────────────────────────────────────────────────────┐
│  payload_sdk_ros1 (核心接口节点)                                │
│  - 飞控数据订阅 (15+ 种数据类型, 最高 400Hz)                      │
│  - 飞行控制 (体轴系速度控制, mavros / 自定义60协议)               │
│  - 感知可视化 (速度/路径/避障 RViz markers, Livox 点云坐标变换)     │
│  - 数据发布 (imu_60, odom, imu, GPS, height, RC)              │
└─────────────────────┬────────────────────────────────────────┘
                      │
              ┌───────▼──────────────────┐
              │       custom_msgs         │
              │───────────────────────────│
              │  flyctrl   (飞控协议)       │
              │  com_package (通信协议)     │
              │  livox_ros  (Livox 激光)   │
              │  jp_device  (JP 激光/雷达)   │
              └───────────────────────────┘
```

**数据流简述：**

1. DJI 飞控通过 PSDK 协议推送传感器数据至机载计算机
2. `payload_sdk_ros1` 订阅 PSDK 数据，转换为 ROS 标准消息格式发布，供上层应用消费

---

## 3. 支持的机型与硬件

根据 [DJI Payload SDK 平台选择文档](https://developer.dji.com/doc/payload-sdk-tutorial/cn/model-instruction/choose-develop-platform.html)，本功能包支持所有接入 DJI PSDK 的无人机，包括但不限于：

- **Matrice 系列**：M350 RTK / M300 RTK
- **Mavic Enterprise 系列**
- 更多最新支持机型请参考 [DJI 官方文档](https://developer.dji.com/doc/payload-sdk-tutorial/cn/quick-start/introduction.html)

### 硬件要求

- **搭载接口**：DJI SkyPort V2 / X-Port / 无人机扩展接口适配器
- **机载计算机**：
  - DJI Manifold 2 / Manifold 3
  - NVIDIA Jetson (AGX Xavier / Orin 等)
  - 任意 x86_64 或 aarch64 Linux 设备
- **操作系统**：Ubuntu 20.04 (推荐, 配合 ROS Noetic)
- **PSDK 库版本**：libpayloadsdk.a (V3.x, 已内置在 `psdk_lib/lib/`)

### PSDK 连接验证

在启动 ROS 节点前，请确保 DJI PSDK 已正确连接（指示灯状态正常，日志无错误）。

---

## 4. 功能特性

### 4.1 飞控数据订阅

通过 `DjiFcSubscription` 模块订阅以下 15+ 种飞行数据：

| 数据项 | PSDK 枚举 | 频率 | 说明 |
|--------|-----------|------|------|
| 融合位置 | `POSITION_FUSED` | 50Hz | LLA（经纬高）→ NEU XYZ 坐标转换 |
| 姿态四元数 | `QUATERNION` | 50Hz | 已转发为 mavros 格式 (`/dji/odom_trans`, `/dji/imu_trans`) |
| 速度 | `VELOCITY` | 50Hz | NEU 坐标系，已转发为 mavros 格式 (提供 NEU/NED/FRD/FLU/FRU 多坐标系) |
| 角速度 (融合) | `ANGULAR_RATE_FUSIONED` | 50Hz | 机体角速度 |
| 加速度 (机体) | `ACCELERATION_BODY` | 10Hz | 机体坐标系加速度 |
| 加速度 (地面) | `ACCELERATION_GROUND` | 10Hz | 地面坐标系加速度 |
| 加速度 (原始) | `ACCELERATION_RAW` | 50Hz | IMU 原始加速度 |
| 融合高度 | `ALTITUDE_FUSED` | 50Hz | 无人机融合海拔高度 |
| 融合高度 (精) | `HEIGHT_FUSION` | 50Hz | 精细融合高度值 |
| GPS 位置 | `GPS_POSITION` | 5Hz | GPS 原始经纬高 |
| GPS 详情 | `GPS_DETAILS` | 5Hz | GPS 卫星数、定位精度等 |
| RTK 连接状态 | `RTK_CONNECT_STATUS` | 5Hz | RTK 基站连接状态 |
| RTK 位置 | `RTK_POSITION` | 5Hz | RTK 定位信息 |
| RTK 速度 | `RTK_VELOCITY` | 5Hz | RTK 速度信息 |
| RTK 偏航 | `RTK_YAW` | 5Hz | RTK 双天线偏航角 |
| 飞行状态 | `STATUS_FLIGHT` | 5Hz | 电机状态、飞行阶段 |
| 显示模式 | `STATUS_DISPLAYMODE` | 5Hz | 飞控工作模式 (P/GPS/A/T 等) |
| 控制设备 | `CONTROL_DEVICE` | 1Hz | 当前控制设备 (遥控器/Offboard) |
| 遥控器数据 | `RC` | 10Hz | 遥控器通道原始值 |
| 遥控器含标志 | `RC_WITH_FLAG_DATA` | 10Hz | 遥控器数据 + 标志位 |
| 避障数据 | `AVOID_DATA` | 50Hz | 各方向障碍物距离，可视化发布 |

**发布的话题：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/guandao1` (可配置) | `com_package/imu_60` | 60 协议导航数据 (位置/速度/姿态) |
| `/dji/odom_trans` | `nav_msgs/Odometry` | 无人机里程计 (mavros 兼容) |
| `/dji/imu_trans` | `sensor_msgs/Imu` | 无人机 IMU 数据 (mavros 兼容) |
| `/dji/gps_init_pos` | `sensor_msgs/NavSatFix` | GPS 初始位置 |
| `/dji/vis` | `visualization_msgs/Marker` | 速度可视化箭头 |
| `/dji/path_vis` | `nav_msgs/Path` | 飞行轨迹可视化 |
| `/dji/vel_ctrl_vis` | `visualization_msgs/Marker` | 控制命令可视化 |
| `/dji/avoid_obs_vis` | `visualization_msgs/MarkerArray` | 避障距离可视化 |
| `/temp_radio` | `std_msgs/Float64` | 融合高度（调试用） |
| `/vel_ctrl_smooth_data` | `geometry_msgs/Twist` | 平滑后控制量 (调试用) |

### 4.2 飞行控制

- **当前支持模式**：体轴系速度控制 (FRD 坐标系：前-右-下)
- **待扩展**：NED 速度控制、NED 位置控制
- **命令输入方式**：
  - `mavros` 类型：通过 `mavros_msgs/PositionTarget` 话题控制
  - `60` 类型：通过 `flyctrl/flyctrl_send` 自定义消息控制

**控制平滑与限速：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_vel_ctrl_smooth` | true | 加速度平滑（限制加速度突变） |
| `enable_vel_ctrl_vel_limit` | true | 速度限幅 |
| `vel_ctrl_vel_limit` | 1.0 m/s | 最大线速度 |
| `vel_ctrl_acc_limit` | 5.0 m/s² | 最大线加速度 |
| `vel_ctrl_yaw_dot_limit` | 30 deg/s | 最大偏航角速度 |
| `vel_ctrl_yaw_dot_dot_limit` | 30 deg/s² | 最大偏航角加速度 |

**心跳超时保护**：持续未收到控制命令时自动急停。

**RC 拨轮切换 Offboard 模式：**

```
右手拨轮向右拨到底，持续 3 秒   →  进入 Offboard 模式（默认体轴系速度控制）
左手拨轮向左拨到底，持续 0.5 秒 →  返回 RC 遥控模式
```

拨轮阈值由源码中 `DJI_RC_GEAR_RIGHT_THR` (9000) 和 `DJI_RC_GEAR_LEFT_THR` (-9000) 定义。

### 4.3 感知与可视化

- **Livox 激光雷达**：订阅 `/livox/lidar` 话题，支持 6-DoF 坐标变换到机体坐标系
- **避障可视化**：以无人机为中心显示各方向障碍物距离 (RViz MarkerArray)
- **飞行状态可视化**：速度矢量 / 航迹 / 控制指令 实时显示
- **GPS 精度监控**：基于 GPS 精度因子自动判断定位可靠度

### 4.4 设备驱动

| 驱动包 | 设备 | 说明 |
|--------|------|------|
| `livox_ros_driver` | Livox 激光雷达系列 | 点云数据接收 (CustomMsg) |
| `jp_device_driver` | JP 激光雷达 | LidarStatus, CustomMsg, CustomPoint |
| `jp_device_driver` | JP 毫米波雷达 | RadarStatus, RadarMsg |

---

## 5. 目录结构

```
src/
├── README.md                                  ← 本文件
├── CMakeLists.txt                              ← catkin 顶层 CMake
│
├── payload_sdk_ros1/                           ← [核心] DJI PSDK ROS1 接口
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/payload_sdk_ros1/
│   │   └── payload_sdk_interface.h             ← PayloadSdkInterface 类声明
│   ├── src/
│   │   ├── dji_ctrl_midware_node.cpp           ← 主节点入口
│   │   ├── payload_sdk_interface.cpp           ← 核心实现 (~1500 行)
│   │   ├── livox_data_trans.cpp                ← Livox 坐标变换独立节点
│   │   └── test.cpp                            ← LLA/XYZ 转换测试
│   ├── launch/
│   │   ├── dji.launch                          ← 主启动文件
│   │   └── rviz.launch / vis.rviz
│   ├── msg/                                    ← 自定义消息 (imu_60, flyctrl_send等)
│   ├── psdk_lib/                               ← DJI PSDK 库文件
│   │   ├── include/                            ← 32 个 PSDK C 头文件
│   │   └── lib/                                ← 多平台静态库 (.a/.lib)
│   └── samples/                                ← DJI 官方 PSDK 示例代码
│
└── custom_msgs/                                ← [消息] 自定义消息与驱动
    ├── com_package/                            ← 通信协议消息 (imu_60, flyctrl 等)
    ├── flyctrl/                                ← 飞控消息 + 60→mavros 转换节点
    ├── livox_ros_driver/                       ← Livox 激光雷达消息定义
    └── jp_device_driver/                       ← JP 激光/毫米波雷达消息与启动文件
```

---

## 6. 依赖项

### 系统库

| 依赖 | 用途 | 必需 |
|------|------|------|
| Eigen3 | 矩阵/向量运算 | 是 |
| PCL | 点云处理 | 是 |
| FFMPEG | PSDK 视频编解码 | 是 |
| OPUS | PSDK 音频编解码 | 是 |
| LIBUSB | PSDK USB 通信 | 是 |
| OpenCV | 图像显示 / UDP 图像传输 | 可选 |

### ROS 包

`roscpp`, `rospy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`,
`message_generation`, `message_runtime`, `tf2_ros`, `pcl_conversions`, `mavros_msgs`

### DJI 库

`libpayloadsdk.a` (已内置，位于 `psdk_lib/lib/`，支持 x86_64 / aarch64 / arm 多平台)

---

## 7. 快速开始

### Step 0: 确认 PSDK 连接正常

确保机载计算机与无人机之间的 PSDK 物理连接正确，串口/USB/网口通信正常。

### Step 1: 编译

```shell
# 创建工作空间
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src

# 克隆仓库
git clone https://github.com/Gaiwqqq/DjiPayloadSDKInterfaceForRos1.git

# 编译
cd ~/catkin_ws
catkin_make
```

编译注意事项：
- 根据平台架构 (x86_64 / aarch64) 自动选择合适的 `libpayloadsdk.a`
- 可设置 `-DBUILD_TEST_CASES_ON=TRUE` 编译测试用例
- 可设置 `-DMEMORY_LEAK_CHECK_ON=TRUE` 启用内存泄漏检测

### Step 2: 启动

```shell
source ~/catkin_ws/devel/setup.bash
roslaunch payload_sdk_ros1 dji.launch
```

启动文件将同时运行：
1. `payload_sdk_ros1_node` — 核心 PSDK 接口节点
2. `livox_trans_node` — Livox 点云坐标变换节点

### Step 3: Offboard 控制模式

```
将遥控器右手拨轮向右拨到底，持续 3 秒   → 进入 Offboard 模式 (体轴系速度控制)
将遥控器左手拨轮向左拨到底，持续 0.5 秒 → 返回 RC 遥控模式
```

---

## 8. 启动参数参考

`dji.launch` 完整参数说明：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `dji/topic_nav_pub` | string | `/guandao1` | IMU60 导航数据发布话题名称 |
| `dji/topic_ctrl_sub` | string | `/flyctrl_send` | 控制命令订阅话题名称 |
| `dji/cmd_type` | string | `"60"` | 控制命令类型：`"mavros"` 或 `"60"` |
| `dji/data_loop_rate` | double | 50.0 | 数据发布频率 (Hz) |
| `dji/gps_accuracy_threshold` | double | 10000.0 | GPS 精度阈值（< 1:理想, 1-2:优秀, 2-5:良好, 5-10:中等, 10-20:一般, >20:弱） |
| `dji/livox_sub_topic` | string | `/livox/lidar` | Livox 激光雷达点云订阅话题 |
| `dji/livox_trans_enable` | bool | false | 是否启用 Livox 点云坐标变换 |
| `dji/enable_livox_frame_tf_pub` | bool | true | 是否发布 Livox 坐标系 TF |
| `dji/enable_vel_ctrl_smooth` | bool | true | 是否启用控制命令平滑 |
| `dji/enable_vel_ctrl_vel_limit` | bool | true | 是否启用速度限幅 |
| `dji/vel_ctrl_vel_limit` | double | 1.0 | 最大线速度 (m/s) |
| `dji/vel_ctrl_acc_limit` | double | 5.0 | 最大线加速度 (m/s²) |
| `dji/vel_ctrl_yaw_dot_limit` | double | 30.0 | 最大偏航角速度 (deg/s) |
| `dji/vel_ctrl_yaw_dot_dot_limit` | double | 30.0 | 最大偏航角加速度 (deg/s²) |
| `dji/livox_trans_matrix/pitch` | double | -15.0 | Livox→机体 俯仰角 (deg) |
| `dji/livox_trans_matrix/roll` | double | 180.0 | Livox→机体 横滚角 (deg) |
| `dji/livox_trans_matrix/yaw` | double | 0.0 | Livox→机体 偏航角 (deg) |
| `dji/livox_trans_matrix/x` | double | 0.0 | Livox→机体 X 平移 (m) |
| `dji/livox_trans_matrix/y` | double | 0.0 | Livox→机体 Y 平移 (m) |
| `dji/livox_trans_matrix/z` | double | 0.0 | Livox→机体 Z 平移 (m) |

---

## 9. 坐标系定义

### 传感器数据坐标系

| 数据项 | DJI 原始坐标系 | 内部处理 | ROS 输出坐标系 |
|--------|---------------|----------|---------------|
| **Position (位置)** | LLA (经度, 纬度, 海拔) | 迭代法转换 LLA→NEU XYZ | **NEU**: 北(X)-东(Y)-天(Z) |
| **Quaternion (姿态)** | (pitch, roll, yaw): **前-左-上** 坐标系, 顺时针为正, 北向 yaw=0 | pitch & yaw 与显示坐标系反向, 需取反 | 60 协议定义一致, 无需转换 |
| **Velocity (速度)** | **NEU**: 北(X)-东(Y)-天(Z) | mavros 格式转换为 NED | NEU (DJI 原始) / NED (mavros) |
| **Acceleration (加速度)** | 机体坐标系 (Body) / 地面坐标系 (Ground) / 原始 (Raw) | 无转换 | 直接转发 |

### 控制坐标系

| 控制模式 | 坐标系 | 轴定义 | 说明 |
|----------|--------|--------|------|
| `OFFBOARD_VEL_BODY` | **FRD** | 前(X)-右(Y)-下(Z) | 体轴系速度控制 (当前已实现) |
| `OFFBOARD_VEL_NED` | **NED** | 北(X)-东(Y)-地(Z) | NED 速度控制 (待实现) |
| `OFFBOARD_POS_NED` | **NED** | 北(X)-东(Y)-地(Z) | NED 位置控制 (待实现) |

### 60 协议与 DJI PSDK 对照

| 参数 | 60 协议定义 | DJI PSDK 定义 | 一致性 |
|------|-----------|---------------|--------|
| 位置 | LLA (经纬高) | LLA (经纬高) | 一致, 无需转换 |
| 姿态 | 前-左-上, 顺时针+, 北 yaw=0 | 前-左-上, 顺时针+, 北 yaw=0 | 一致, 无需转换 |
| 速度 (体轴) | NEU (东北天) | NEU (东北天) | 一致, 无需转换 |

---

## 10. 扩展开发指南

### 添加新的 DJI 数据订阅

1. 在 `payload_sdk_interface.h` 中声明对应的 `T_DjiFcSubscription*` 数据成员
2. 在 `payload_sdk_interface.cpp` 构造函数中调用 `djiCreateSubscription()`，指定话题枚举和频率
3. 编写数据处理回调函数，填充 ROS 消息并发布

参见 [DJI PSDK API 参考 - 飞控订阅](https://developer.dji.com/doc/payload-sdk-api-reference/cn/) 了解所有可用的 `E_DjiFcSubscriptionTopic` 枚举值。

### 添加新的控制模式

在 `PayloadSdkInterface::ctrlMode` 枚举中添加新模式，参考 `DJI_FLIGHT_CONTROLLER_*` 系列 API：
- `DjiFlightController_EmergencyBrakeAction()` — 紧急刹车
- `DjiFlightController_JoystickAction()` — 摇杆控制
- `DjiFlightController_SetHorizontalVoCtrlMode()` — 水平速度控制模式
- `DjiFlightController_SetVerticalVoCtrlMode()` — 垂直速度控制模式
- `DjiFlightController_SetYawVoCtrlMode()` — 偏航角速度控制模式
- `DjiFlightController_SetHorizontalPosCtrlMode()` — 水平位置控制模式
- `DjiFlightController_SetVerticalPosCtrlMode()` — 垂直位置控制模式
- `DjiFlightController_SetYawPosCtrlMode()` — 偏航角位置控制模式

---

## 11. 参考链接

- [DJI Payload SDK 官方 GitHub](https://github.com/dji-sdk/Payload-SDK)
- [DJI PSDK 开发教程 (中文)](https://developer.dji.com/doc/payload-sdk-tutorial/cn/)
- [DJI PSDK API 参考 (中文)](https://developer.dji.com/doc/payload-sdk-api-reference/cn/)
- [DJI SDK 开发者论坛 (中文)](https://djisdksupport.zendesk.com/hc/zh-cn/community/topics)
- [DJI SDK 开发者论坛 (英文)](https://djisdksupport.zendesk.com/hc/en-us/community/topics)
- [DJI SDK 技术支持](https://djisdksupport.zendesk.com/hc/zh-cn/requests/new)
- [ROS Wiki](http://wiki.ros.org/)

---

## 免责声明

本功能包仅提供基础接口与示例代码，部分功能可能未经充分飞行测试。**使用者应对自身行为及其后果承担全部责任。** 在实际飞行前，请务必在仿真或自稳环境下充分测试。

## 维护者

Weiqi Gai (gwq) 等

## 许可证

待定 (请参考 `payload_sdk_ros1/package.xml` 与 DJI PSDK [LICENSE](https://github.com/dji-sdk/Payload-SDK/blob/master/LICENSE.txt))
