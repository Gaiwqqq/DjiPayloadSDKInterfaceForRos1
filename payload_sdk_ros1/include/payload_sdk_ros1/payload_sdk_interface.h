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


#define INFO_MSG(str)        do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str)    do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str)  do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str)   do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

class PayloadSdkInterface{
public:
    PayloadSdkInterface(ros::NodeHandle &nh);
    ~PayloadSdkInterface();

private:
    ros::NodeHandle nh_;
    ros::Timer      dji_data_read_timer_;

    T_DjiReturnCode                      djiStat_;
    T_DjiOsalHandler                     *osalHandler_;
    T_DjiFcSubscriptionQuaternion        dji_quaternion_data_{0};
    T_DjiFcSubscriptionVelocity          dji_velocity_data_{0};
    T_DjiDataTimestamp                   dji_timestamp_data_{0};
    T_DjiFcSubscriptionGpsPosition       dji_gps_position_data_{0};
    T_DjiFcSubscriptionPositionFused     dji_position_fused_data_{0};
    T_DjiFcSubscriptionSingleBatteryInfo dji_single_battery_info_data_{0};

    // data transmission
    Eigen::Vector3d              quaternion_data_;        // (pitch, roll, yaw)
    Eigen::Vector3d              velocity_data_;          // (vx, vy, vz)
    Eigen::Vector3d              gps_position_data_;      // (latitude, longitude, altitude)
    Eigen::Vector3d              position_fused_data_;    //

    // counters
    uint32_t                      quaternion_recv_counter_;

    // flags
    bool                          is_quaternion_disp_;

    // callbacks
    void djiDataReadCallback(const ros::TimerEvent& event);

    // functions
    bool djiCreateSubscription(std::string topic_name, E_DjiFcSubscriptionTopic topic,
                                E_DjiDataSubscriptionTopicFreq frequency,
                                DjiReceiveDataOfTopicCallback callback);

};


#endif //PAYLOAD_SDK_INTERFACE_H
