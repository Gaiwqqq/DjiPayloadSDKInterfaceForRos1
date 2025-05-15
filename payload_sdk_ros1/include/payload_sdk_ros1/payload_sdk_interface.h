//
// Created by gwq on 25-4-14.
//

#ifndef PAYLOAD_SDK_INTERFACE_H
#define PAYLOAD_SDK_INTERFACE_H

#include "ros/ros.h"
#include <Eigen/Eigen>

#include <liveview/test_liveview_entry.hpp>
#include <perception/test_perception_entry.hpp>
#include <flight_control/test_flight_control.h>
#include <gimbal/test_gimbal_entry.hpp>
#include "application.hpp"
#include "fc_subscription/test_fc_subscription.h"
#include <gimbal_emu/test_payload_gimbal_emu.h>
#include <camera_emu/test_payload_cam_emu_media.h>
#include <camera_emu/test_payload_cam_emu_base.h>
#include <dji_logger.h>
#include "widget/test_widget.h"
#include "widget/test_widget_speaker.h"
#include <power_management/test_power_management.h>
#include "data_transmission/test_data_transmission.h"
#include <flight_controller/test_flight_controller_entry.h>
#include <positioning/test_positioning.h>
#include <hms_manager/hms_manager_entry.h>
#include "camera_manager/test_camera_manager_entry.h"

#include <utils/util_misc.h>
#include <dji_flight_controller.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <com_package/imu_60.h>
#include <flyctrl/flyctrl_send.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#define INFO_MSG(str)        do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str)    do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str)  do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str)   do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_PURPLE(str) do {std::cout << "\033[35m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_CYAN(str)   do {std::cout << "\033[36m" << str << "\033[0m" << std::endl; } while(false)

#define DJI_RC_GEAR_RIGHT_THR 9000
#define DJI_RC_GEAR_LEFT_THR  -9000

/*
 * @ 关于坐标系的说明：
 * 1. Position (DJI 坐标系为经纬高)
 *    1.1. LLA转换XYZ后的坐标系为东北天，与显示坐标系西北天不符，转换后需要变为 -y
 *    1.2  LLA与60协议相同，无需转换 （经纬高）
 * 2. Quaternion ()
 *    2.1 DJI坐标系的pitch和 yaw与显示坐标系反向，需要变为-pitch & -yaw （todo 未验证）
 *    2.2 (pitch, roll, yaw)定义与60协议相同，无需转换  (前左上坐标系， 顺时针为+， 北向yaw=0)
 * 3. Acceleration (DJI 坐标系为todo)
 *    3.1 todo 未验证
 * 4. Velocity (DJI 坐标系为NEU)
 *    4.1 mavros 坐标系为 NED
 *    4.2 60 body-vel 坐标系为东北天, 无需转换
 * */

class PayloadSdkInterface{
public:
  enum CtrlDevice{
    CTRL_DEVICE_RC       = 0,
    CTRL_DEVICE_OFFBOARD = 1
  };
  enum ctrlMode{
    NOT_SET           = 0,
    OFFBOARD_VEL_BODY = 1,
    OFFBOARD_VEL_NED  = 2,
    OFFBOARD_POS_NED  = 3
  };
  typedef std::shared_ptr<PayloadSdkInterface> Ptr;
  typedef std::unique_ptr<PayloadSdkInterface> UNI_Ptr;
  PayloadSdkInterface(ros::NodeHandle &nh, T_DjiOsalHandler *osal_handler);
  ~PayloadSdkInterface();

private:
  ros::NodeHandle nh_;
  ros::Timer      dji_data_read_timer_, dji_flyctrl_pub_timer_;
  ros::Subscriber ctrl_cmd_sub_, offboard_switch_sub_, custom_60_cmd_sub_;
  ros::Publisher  imu_60_pub_, mimicking_flight_60_height_pub_, odom_trans_pub_, imu_trans_pub_;
  ros::Publisher  vis_pub_, path_vis_pub_;
  ros::Subscriber livox_sub_;
  ros::Publisher  livox_pub_;

  // debug pub
  ros::Publisher  vel_ctrl_smooth_data_pub_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  T_DjiReturnCode                      djiStat_;
  T_DjiOsalHandler                     *dji_osal_handler_;
  T_DjiFcSubscriptionAccelerationBody  dji_acc_body_data_{0};
  T_DjiFcSubscriptionAccelerationRaw   dji_acc_raw_data_{0};
  T_DjiFcSubscriptionAccelerationGround dji_acc_ground_data_{0};
  T_DjiFcSubscriptionQuaternion        dji_quaternion_data_{0};
  T_DjiFcSubscriptionVelocity          dji_velocity_data_{0};
  T_DjiFcSubscriptionAltitudeFused     dji_altitude_fused_data_{0};
  T_DjiDataTimestamp                   dji_timestamp_data_{0};
  T_DjiFcSubscriptionGpsPosition       dji_gps_position_data_{0};
  T_DjiFcSubscriptionPositionFused     dji_position_fused_data_{0};
  T_DjiFcSubscriptionSingleBatteryInfo dji_single_battery_info_data_{0};
  T_DjiFcSubscriptionFlightStatus      dji_flight_status_data_{0};
  T_DjiFcSubscriptionDisplaymode       dji_flight_mode_data_{0};
  T_DjiFcSubscriptionAngularRateFusioned dji_angular_rate_fused_data_{0};
  T_DjiFcSubscriptionGpsDetails        dji_gps_details_data_{0};
  T_DjiFcSubscriptionControlDevice     dji_ctrl_device_data_{0};
  T_DjiFcSubscriptionHeightFusion      dji_height_fusion_data_{0};

  T_DjiFcSubscriptionRTKConnectStatus  dji_rtk_connection_stat_data_{0};
  T_DjiFcSubscriptionRtkPosition       dji_rtk_pos_data_{0};
  T_DjiFcSubscriptionRtkVelocity       dji_rtk_vel_data_{0};
  T_DjiFcSubscriptionRtkYaw            dji_rtk_yaw_data_{0};

  T_DjiFcSubscriptionRC                dji_rc_data_{0};
  T_DjiFcSubscriptionRCWithFlagData    dji_rc_with_flag_data_{0};

  // ros msgs
  mavros_msgs::PositionTarget          mavros_cmd_data_recv_;
  flyctrl::flyctrl_send                custom_60_cmd_data_recv_;

  // data transmission
  Eigen::Vector3d              acc_body_data_, acc_ground_data_, acc_raw_data_; // (ax, ay, az)
  Eigen::Vector3d              angular_rate_fused_data_; // (wx, wy, wz)
  Eigen::Vector3d              quaternion_data_;         // (pitch, roll, yaw)
  Eigen::Vector3d              velocity_data_neu_, velocity_data_neu_vis_; // (vx, vy, vz)
  Eigen::Vector3d              velocity_data_frd_, velocity_data_frd_vis_; // (vx, vy, vz)
  Eigen::Vector3d              velocity_data_flu_;       // (vx, vy, vz)
  Eigen::Vector3d              gps_position_data_;       // (latitude, longitude, altitude)
  Eigen::Vector3d              position_fused_data_;     // (latitude, longitude, altitude)
  double                       altitude_fused_data_;     //
  Eigen::Quaterniond           quaternion_world_;        // (qx, qy, qz, qw)

  Eigen::Vector3d              neu_pos_init_;            // (latitude, longitude, altitude)
  Eigen::Vector3d              xyz_pos_neu_;             // (x, y, z)
  double                       gps_pos_accuracy_;        // <1: 理想, 1-2: 优秀, 2-5: 良好, 5-10: 中等, 10-20: 一般, >20: 弱。

  // ctrl cmd data
  Eigen::Vector4d              vel_ctrl_cmd_data_frd_raw_, vel_ctrl_cmd_data_frd_fix_;   // (vx, vy, vz, yaw_rate) -> FRD coordinate
  nav_msgs::Path               path_vis_data_;

  // counters
  uint32_t                      quaternion_recv_counter_;

  // flags
  bool                          is_quaternion_disp_, ctrl_cmd_heartbeat_ready_, position_fused_ready_flag_;
  bool                          is_gps_convergent_, dji_ctrl_first_init_, dji_ctrl_init_success_;
  CtrlDevice                    cur_ctrl_device_;
  ctrlMode                      cur_ctrl_mode_;
  uint16_t                      mavros_cmd_type_mask_velctrl_only_;
  ros::Time                     last_ctrl_cmd_time_, last_pos_fused_recv_time_;
  ros::Time                     last_dji_cmd_pub_time_;
  bool                          gps_ready_;
  string                        ctrl_cmd_type_;

  ros::Time                     gear_change_start_time_;
  unsigned int                  gear_moniting_phase_{0};   // 0 -> 1 -> 2 -> 0

  bool                          vel_ctrl_smooth_flag_;

  // throttles
  double                        _gps_accuracy_thres;
  double                        _data_loop_rate;
  Eigen::Matrix4d               _livox2body_matrix;
  double                        _max_ctrl_acc, _max_ctrl_yaw_dot_dot;

  // callbacks
  void djiDataReadCallback(const ros::TimerEvent& event);
  void mavrosCmdCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);
  void custom60CmdCallback(const flyctrl::flyctrl_send::ConstPtr &msg);
  void offboardSwitchCallback(const std_msgs::Int8::ConstPtr& msg);
  void djiFlyCtrlPubCallback(const ros::TimerEvent& event);
  void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg);
  void fullMotionStop();
  void velCtrlSmooth(const ros::Time &cur_t);

  void feedPositionDataProcess();
  void feedGPSDetailsDataProcess();
  void feedVelDataProcess();
  void feedQuaternionDataProcess();
  void feedRCDataProcess();

  void publishImu60Data();
  void publishOdomData();
  void publishImuMavrosData();

  // functions
  bool djiCreateSubscription(std::string topic_name, E_DjiFcSubscriptionTopic topic,
                              E_DjiDataSubscriptionTopicFreq frequency,
                              DjiReceiveDataOfTopicCallback callback);
  void djiDestroySubscription(std::string topic_name, E_DjiFcSubscriptionTopic topic);
  bool switchCtrlDevice(CtrlDevice device);
  template<typename T>
  void readParam(std::string param_name, T &param_val, T default_val);

  // visualization
  void drawVel();
  void drawRangeCircles();
  void drawPath();

  void livoxTransInit();
  Eigen::Vector3d XYZ2LLA(const Eigen::Vector3d& xyz);
  Eigen::Vector3d LLA2XYZ(const Eigen::Vector3d& lla);
  Eigen::Quaterniond euler2Quaternion(const Eigen::Vector3d& euler); // (pitch, roll, yaw)

};

#endif //PAYLOAD_SDK_INTERFACE_H
