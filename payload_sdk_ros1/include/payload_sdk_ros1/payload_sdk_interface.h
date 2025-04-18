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

#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Int8.h>
#include <payload_sdk_ros1/imu_60.h>

#define INFO_MSG(str)        do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str)    do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str)  do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str)   do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

class PayloadSdkInterface{
public:
  enum ctrlDevice{
    CTRL_DEVICE_RC = 0,
    CTRL_DEVICE_OFFBOARD = 1
  };
  enum ctrlMode{
    NOT_SET = 0,
    OFFBOARD_VEL_BODY = 1,
    OFFBOARD_VEL_NED = 2,
    OFFBOARD_POS_NED = 3
  };
  typedef std::shared_ptr<PayloadSdkInterface> Ptr;
  PayloadSdkInterface(ros::NodeHandle &nh, T_DjiOsalHandler *osal_handler);
  ~PayloadSdkInterface();

private:
  ros::NodeHandle nh_;
  ros::Timer      dji_data_read_timer_, dji_flyctrl_pub_timer_;
  ros::Subscriber mavros_cmd_sub_, offboard_switch_sub_;
  ros::Publisher  imu_60_pub_, mimicking_flight_hight_pub_;

  T_DjiReturnCode                      djiStat_;
  T_DjiOsalHandler                     *dji_osal_handler_;
  T_DjiFcSubscriptionAccelerationBody  dji_acc_body_data_{0};
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

  // ros msgs
  mavros_msgs::PositionTarget          mavros_cmd_data_recv_;

  // data transmission
  Eigen::Vector3d              acc_body_data_;           // (ax, ay, az)
  Eigen::Vector3d              angular_rate_fused_data_; // (wx, wy, wz)
  Eigen::Vector3d              quaternion_data_;         // (pitch, roll, yaw)
  Eigen::Vector3d              velocity_data_;           // (vx, vy, vz)
  Eigen::Vector3d              gps_position_data_;       // (latitude, longitude, altitude)
  Eigen::Vector3d              position_fused_data_;     // (latitude, longitude, altitude)
  double                       altitude_fused_data_;     //

  // counters
  uint32_t                      quaternion_recv_counter_;

  // flags
  bool                          is_quaternion_disp_, mavros_cmd_heartbeat_ready_, position_fused_ready_flag_, dji_ctrl_first_init_;
  ctrlDevice                    cur_ctrl_device_;
  ctrlMode                      cur_ctrl_mode_;
  uint16_t                      mavros_cmd_type_mask_velctrl_only_;
  ros::Time                     last_mavros_cmd_time_, last_pos_fused_recv_time_;

  // callbacks
  void djiDataReadCallback(const ros::TimerEvent& event);
  void mavrosCmdCallback(const mavros_msgs::PositionTarget::ConstPtr& msg);
  void offboardSwitchCallback(const std_msgs::Int8::ConstPtr& msg);
  void djiFlyCtrlPubCallback(const ros::TimerEvent& event);

  void feedPositionDataProcess();

  void publishImu60Data();

  // functions
  bool djiCreateSubscription(std::string topic_name, E_DjiFcSubscriptionTopic topic,
                              E_DjiDataSubscriptionTopicFreq frequency,
                              DjiReceiveDataOfTopicCallback callback);
  void djiDestroySubscription(std::string topic_name, E_DjiFcSubscriptionTopic topic);
  bool switchCtrlDevice(ctrlDevice device);
  template<typename T>
  void readParam(std::string param_name, T &param_val, T default_val);
};


#endif //PAYLOAD_SDK_INTERFACE_H
