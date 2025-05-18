# 🤗 DJI-ROS [Dji-Payload-SDK-Interface-For-ROS1]  使用简介 🤗

DJI-ROS 这一功能包，是基于 DJI-Payload-SDK-Interface 功能包，为 ROS1 平台提供了 DJI 无人机的相关功能接口。

**✅** 重点是提供一个无需额外配置编译的集成功能包，**使得开发者可在熟练使用ROS的情况下根据本代码给出的实例和框架迅速完成对应功能的开发**

**✅** 注意：本功能包仅提供简单实例，部分功能可能未经过飞行测试，**不对使用者的任何行为以及后果负责**。

<hr style="border: 2px solid green;">

**支持的机型：**
- 任何支持DJI Payload SDK的无人机，详见[DJI-Payload-SDK文档](https://developer.dji.com/doc/payload-sdk-tutorial/cn/model-instruction/choose-develop-platform.html)
  - M350 / M300 ...
  - DJI-Mavic Enterprise series
---
**目前支持的功能如下：**(仅展示部分，可根据DJI文档中给定的消息订阅类型自行扩展)
- 🥰 无人机各类型消息订阅
  - 融合位置 [fused_position] → **已转发为mavros格式
  - 姿态角四元数 [quaterion] → **已转发为mavros格式
  - 速度 [velocity] → **已转发为mavros格式
  - 融合高度 [fused_height]
  - 飞行状态 [flight_status]
  - GPS原始数据 [gps_raw]
  - RTK原始数据 [rtk_raw]
  - 遥控器原始数据 [rc_raw]
---
- 🥰 无人机飞行控制
  - 自定义话题输入，根据自定义消息控制无人机飞行 （目前仅开发了体轴系vel控制）
  - 使用mavros给定话题控制无人机飞行（目前仅开发了体轴系vel控制）

<hr style="border: 2px solid green;">
## 0 快速上手

### step0. Make sure your payload-sdk is correctly connected

### step1. Build
```shell
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/Gaiwqqq/DjiPayloadSDKInterfaceForRos1.git
cd ..
catkin_make
```    
### Step2. Run (use launch file)
```shell
source devel/setup.bash 
roslaunch payload_sdk_ros1 dji.launch
```
<hr style="border: 2px solid green;">
## 1 坐标系定义


- ✅ 关于数据坐标系的说明：
  - ✨ Position (DJI 坐标系为经纬高)
    - 1 LLA转换XYZ后的坐标系为东北天，与显示坐标系西北天不符，转换后需要变为 -y
    - 2 LLA与60协议相同，无需转换 （经纬高）
  - ✨ Quaternion ()
    - 1 DJI坐标系的pitch和 yaw与显示坐标系反向，需要变为-pitch & -yaw （todo 未验证）
    - 2 (pitch, roll, yaw)定义与60协议相同，无需转换  (前左上坐标系， 顺时针为+， 北向yaw=0)
  - ✨ Acceleration (DJI 坐标系为todo)
    - 1 todo 未验证
  - ✨ Velocity (DJI 坐标系为NEU)
    - 1 mavros 坐标系为 NED
    - 2 60 body-vel 坐标系为东北天, 无需转换
---
- ✅ 关于控制坐标系的说明：
  - 体坐标系速度控制 → 前右下坐标系 （z轴逆时针yaw为+， 北向yaw=0，角速度单位为deg/s）
