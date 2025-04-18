#include "../include/payload_sdk_ros1/payload_sdk_interface.h"



PayloadSdkInterface::PayloadSdkInterface(ros::NodeHandle &nh, T_DjiOsalHandler *osal_handler){
  // ros init
  nh_ = nh;

  // -------------------- DJI init --------------------------//
  dji_osal_handler_ = osal_handler;

  dji_acc_body_data_            = {0};
  dji_quaternion_data_          = {0};
  dji_velocity_data_            = {0};
  dji_gps_position_data_        = {0};
  dji_timestamp_data_           = {0};
  dji_single_battery_info_data_ = {0};
  dji_position_fused_data_      = {0};
  dji_altitude_fused_data_      = {0};
  dji_flight_status_data_       = {0};
  dji_flight_mode_data_         = {0};
  dji_angular_rate_fused_data_  = {0};

  quaternion_recv_counter_      = 0;
  is_quaternion_disp_           = false;
  cur_ctrl_device_              = CTRL_DEVICE_RC;
  mavros_cmd_heartbeat_ready_   = false;
  position_fused_ready_flag_    = false;
  dji_ctrl_first_init_          = true;

  last_mavros_cmd_time_     = ros::Time::now();
  last_pos_fused_recv_time_ = last_mavros_cmd_time_;

  mavros_cmd_type_mask_velctrl_only_ =
    mavros_msgs::PositionTarget::IGNORE_PX  | mavros_msgs::PositionTarget::IGNORE_PY  | mavros_msgs::PositionTarget::IGNORE_PZ  |
    mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
    mavros_msgs::PositionTarget::IGNORE_YAW;

  INFO_MSG("[DJI]: Payload SDK init success, do topic init...");

  djiStat_ = DjiFcSubscription_Init();
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    INFO_MSG_RED("[DJI]: init data subscription module error, quit program");
    return;
  }

  std::map<int, E_DjiDataSubscriptionTopicFreq> freq_map;
  freq_map[200] = DJI_DATA_SUBSCRIPTION_TOPIC_200_HZ;
  freq_map[100] = DJI_DATA_SUBSCRIPTION_TOPIC_100_HZ;
  freq_map[50]  = DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ;
  freq_map[10]  = DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ;
  freq_map[5]   = DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ;
  freq_map[1]   = DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ;

  bool dji_init_success = true;
  dji_init_success =
    djiCreateSubscription("acc_body", DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY, freq_map[100], nullptr);
  dji_init_success =
    djiCreateSubscription("angular_rate_fused", DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED, freq_map[100], nullptr);
  dji_init_success =
    djiCreateSubscription("quaternion", DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, freq_map[100], nullptr);
  dji_init_success =
    djiCreateSubscription("pos_fusion", DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, freq_map[100], nullptr);
  dji_init_success =
    djiCreateSubscription("altitude_fused", DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED, freq_map[100], nullptr);
  dji_init_success =
    djiCreateSubscription("velocity", DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, freq_map[50], nullptr);
  dji_init_success =
    djiCreateSubscription("gps_position", DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, freq_map[5], nullptr);
  dji_init_success =
    djiCreateSubscription("flight_status", DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, freq_map[5], nullptr);
  dji_init_success =
    djiCreateSubscription("flight_mode", DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, freq_map[5], nullptr);

  if (dji_init_success){
    dji_data_read_timer_ = nh_.createTimer(ros::Duration(1.0 / 50.0), &PayloadSdkInterface::djiDataReadCallback, this);
    dji_flyctrl_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / 50.0), &PayloadSdkInterface::djiFlyCtrlPubCallback, this);
    dji_flyctrl_pub_timer_.stop();
    INFO_MSG_GREEN("[DJI]: Payload SDK init success, do topic init success !");
    INFO_MSG_GREEN("[DJI]: Subscribe to topics: quaternion, velocity, gps_position, pos_fusion");
    INFO_MSG_GREEN("[DJI]: recv frequency : " << 50 << " Hz");
  }else{
    INFO_MSG_RED("[DJI]: Payload SDK init failed, do topic init failed !");
    INFO_MSG_RED("[DJI]: Do NOT launch timer! Quit program");
    return ;
  }

  // -------------------- ros init --------------------------//
  std::string topic_nav_pub, topic_mavros_sub;
  readParam<std::string>("dji/topic_nav_pub", topic_nav_pub, "/mavros/nav_msgs");
  readParam<std::string>("dji/topic_mavros_sub", topic_mavros_sub, "/mavros/setpoint_raw/local");

  mavros_cmd_sub_      = nh_.subscribe(topic_mavros_sub, 2, &PayloadSdkInterface::mavrosCmdCallback,
                                        this , ros::TransportHints().tcpNoDelay());
  offboard_switch_sub_ = nh_.subscribe("/dji/offboard_switch", 2, &PayloadSdkInterface::offboardSwitchCallback,
                                        this, ros::TransportHints().tcpNoDelay());
  imu_60_pub_          = nh_.advertise<payload_sdk_ros1::imu_60>(topic_nav_pub, 2);
}

PayloadSdkInterface::~PayloadSdkInterface(){

  INFO_MSG("[DJI]: Payload SDK interface deconstruct");
  INFO_MSG("[DJI]: Destory flight controller");

  if (cur_ctrl_device_ == CTRL_DEVICE_OFFBOARD)
    switchCtrlDevice(CTRL_DEVICE_RC);

  djiStat_ = DjiFlightController_DeInit();
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                   djiStat_);
  }

  djiDestroySubscription("acc_body", DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY);
  djiDestroySubscription("angular_rate_fused", DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED);
  djiDestroySubscription("quaternion", DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
  djiDestroySubscription("pos_fusion", DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED);
  djiDestroySubscription("altitude_fused", DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED);
  djiDestroySubscription("velocity", DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
  djiDestroySubscription("gps_position", DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);
  djiDestroySubscription("flight_status", DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT);
  djiDestroySubscription("flight_mode", DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE);
  INFO_MSG("[DJI]: Destoried all subscription topics");

  djiStat_ = DjiFcSubscription_DeInit();
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Deinit data subscription module failed, error code:0x%08llX",
                   djiStat_);
  }
  INFO_MSG("[DJI]: Destoried data subscription module");
  INFO_MSG("[DJI]: Payload SDK interface deconstruct success... Goodbye!");
}

void PayloadSdkInterface::djiDataReadCallback(const ros::TimerEvent& event){
  // Acc body
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY,
                                                (uint8_t *) &dji_acc_body_data_,
                                                sizeof(T_DjiFcSubscriptionAccelerationBody),
                                                &dji_timestamp_data_);

  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get acc body data error, timestamp: "
                  << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else
   acc_body_data_ = Eigen::Vector3d(dji_acc_body_data_.x, dji_acc_body_data_.y, dji_acc_body_data_.z);

  // angular rate fused
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED,
                                                (uint8_t *) &dji_angular_rate_fused_data_,
                                                sizeof(T_DjiFcSubscriptionAngularRateFusioned),
                                                &dji_timestamp_data_);

  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get angular rate fused data error, timestamp: "
                  << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else
    angular_rate_fused_data_ = Eigen::Vector3d(dji_angular_rate_fused_data_.x, dji_angular_rate_fused_data_.y, dji_angular_rate_fused_data_.z);

  // Vel
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                  (uint8_t *) &dji_velocity_data_,
                                                  sizeof(T_DjiFcSubscriptionVelocity),
                                                  &dji_timestamp_data_);

  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get velocity data error, timestamp: "
                  << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else
    velocity_data_ = Eigen::Vector3d(dji_velocity_data_.data.x, dji_velocity_data_.data.y, dji_velocity_data_.data.z);

  // Quaternion
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                  (uint8_t *) &dji_quaternion_data_,
                                                  sizeof(T_DjiFcSubscriptionQuaternion),
                                                  &dji_timestamp_data_);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get quaternion data error, timestamp: "
                  << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else{
    T_DjiFcSubscriptionQuaternion quaternion = dji_quaternion_data_;
    double pitch = (dji_f64_t) asinf(-2 * quaternion.q1 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q2) * 57.3;
    double roll = (dji_f64_t) atan2f(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1,
                             -2 * quaternion.q1 * quaternion.q1 - 2 * quaternion.q2 * quaternion.q2 + 1) * 57.3;
    double yaw = (dji_f64_t) atan2f(2 * quaternion.q1 * quaternion.q2 + 2 * quaternion.q0 * quaternion.q3,
                             -2 * quaternion.q2 * quaternion.q2 - 2 * quaternion.q3 * quaternion.q3 + 1) * 57.3;
    quaternion_data_ = Eigen::Vector3d(pitch, roll, yaw);
  }

  // GPS position
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                  (uint8_t *) &dji_gps_position_data_,
                                                  sizeof(T_DjiFcSubscriptionGpsPosition),
                                                  &dji_timestamp_data_);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get gps position data error, timestamp: "
                  << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else
    gps_position_data_ = Eigen::Vector3d(dji_gps_position_data_.x, dji_gps_position_data_.y, dji_gps_position_data_.z);

  // Position fused
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                 (uint8_t *) &dji_position_fused_data_,
                                                 sizeof(T_DjiFcSubscriptionPositionFused),
                                                 &dji_timestamp_data_);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get position fused data error, timestamp: "
                  << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else{
    feedPositionDataProcess();
  }
// single battery info
  // altitude fused
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                 (uint8_t *) &dji_altitude_fused_data_,
                                                 sizeof(T_DjiFcSubscriptionAltitudeFused),
                                                 &dji_timestamp_data_);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get altitude fused data error, timestamp: "
                  << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else
    altitude_fused_data_ = dji_altitude_fused_data_;

  // flight status
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                 (uint8_t *) &dji_flight_status_data_,
                                                 sizeof(T_DjiFcSubscriptionFlightStatus),
                                                 &dji_timestamp_data_);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get flight status data error, timestamp: "
                  << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }

  // flight mode
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                 (uint8_t *) &dji_flight_mode_data_,
                                                 sizeof(T_DjiFcSubscriptionDisplaymode),
                                                 &dji_timestamp_data_);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get flight mode data error, timestamp: "
                  << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }

  // ----------------------- ROS publish -----------------------------//
  publishImu60Data();
}

void PayloadSdkInterface::djiFlyCtrlPubCallback(const ros::TimerEvent& event){
  if (!mavros_cmd_heartbeat_ready_) return;
  if (cur_ctrl_device_ != CTRL_DEVICE_OFFBOARD) return;
  if (cur_ctrl_mode_ == NOT_SET) return ;

  if (cur_ctrl_mode_ == OFFBOARD_VEL_BODY){
    if (mavros_cmd_data_recv_.type_mask != mavros_cmd_type_mask_velctrl_only_ ||
        mavros_cmd_data_recv_.coordinate_frame != mavros_msgs::PositionTarget::FRAME_BODY_NED){
      INFO_MSG_RED("[DJI]: Warning, mavros cmd data type mask not match, only velctrl data is accepted !");
      return;
    }
    T_DjiFlightControllerJoystickCommand joystick_cmd =
      {static_cast<dji_f32_t>(mavros_cmd_data_recv_.velocity.x),
       static_cast<dji_f32_t>(mavros_cmd_data_recv_.velocity.y),
       static_cast<dji_f32_t>(mavros_cmd_data_recv_.velocity.z),
       static_cast<dji_f32_t>(mavros_cmd_data_recv_.yaw_rate)};
    DjiFlightController_ExecuteJoystickAction(joystick_cmd);
  }
}

void PayloadSdkInterface::mavrosCmdCallback(const mavros_msgs::PositionTarget::ConstPtr& msg){
  ros::Time cur_time    = ros::Time::now();
  double time_duration  = (cur_time - last_mavros_cmd_time_).toSec();
  last_mavros_cmd_time_ = cur_time;
  if (time_duration > 0.5 && mavros_cmd_heartbeat_ready_){
    INFO_MSG_RED("[DJI]: Warning, mavros cmd data not received in 200ms, lost heartbeat !");
    mavros_cmd_heartbeat_ready_ = false;
  }
  else if (time_duration < 0.5 && !mavros_cmd_heartbeat_ready_){
    INFO_MSG_GREEN("[DJI]: mavros cmd data heartbeat recovered !");
    mavros_cmd_heartbeat_ready_ = true;
  }

  mavros_cmd_data_recv_ = *msg;
}

void PayloadSdkInterface::offboardSwitchCallback(const std_msgs::Int8::ConstPtr& msg){
  std_msgs::Int8 mode = *msg;
  if (mode.data != 0 && cur_ctrl_device_ == CTRL_DEVICE_RC){
    switchCtrlDevice(CTRL_DEVICE_OFFBOARD);
    if (mode.data == 1){
      cur_ctrl_mode_ = OFFBOARD_VEL_BODY;
      T_DjiFlightControllerJoystickMode joystick_mode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE
      };
      DjiFlightController_SetJoystickMode(joystick_mode);
      dji_flyctrl_pub_timer_.start();
    }
    else if (mode.data == 2){
      cur_ctrl_mode_ = OFFBOARD_VEL_NED;
    }
    else if (mode.data == 3){
      cur_ctrl_mode_ = OFFBOARD_POS_NED;
    }
  }
  else if (mode.data == 0 && cur_ctrl_device_ == CTRL_DEVICE_OFFBOARD){
    switchCtrlDevice(CTRL_DEVICE_RC);
    cur_ctrl_mode_ = NOT_SET;
  }
}

void PayloadSdkInterface::publishImu60Data(){
  payload_sdk_ros1::imu_60 imu60_msg;
  ros::WallTime t = ros::WallTime::now();
  imu60_msg.header.stamp = ros::Time(t.sec, t.nsec);

  imu60_msg.lat       = static_cast<float>(position_fused_data_.x());
  imu60_msg.lon       = static_cast<float>(position_fused_data_.y());
  imu60_msg.alt       = static_cast<float>(position_fused_data_.z());
  imu60_msg.Vx        = static_cast<float>(velocity_data_.x());
  imu60_msg.Vy        = static_cast<float>(velocity_data_.y());
  imu60_msg.Vz        = static_cast<float>(velocity_data_.z());

  // todo: 角度转换存疑，不确定 ！！！！
  imu60_msg.Pitch     = static_cast<float>(quaternion_data_.x());
  imu60_msg.Roll      = static_cast<float>(quaternion_data_.y());
  imu60_msg.Yaw       = static_cast<float>(quaternion_data_.z());
  imu60_msg.RollRate  = static_cast<float>(angular_rate_fused_data_.x());
  imu60_msg.PitchRate = static_cast<float>(angular_rate_fused_data_.y());
  imu60_msg.YawRate   = static_cast<float>(angular_rate_fused_data_.z());
  imu60_msg.Ax        = static_cast<float>(acc_body_data_.x());
  imu60_msg.Ay        = static_cast<float>(acc_body_data_.y());
  imu60_msg.Az        = static_cast<float>(acc_body_data_.z());

  imu60_msg.SensorStatus = 25;
  imu60_msg.WorkStatus   = 8;
  imu60_msg.NaviStatus   = 9;

  imu_60_pub_.publish(imu60_msg);
}

void PayloadSdkInterface::feedPositionDataProcess(){
  position_fused_data_ = Eigen::Vector3d(dji_position_fused_data_.latitude, dji_position_fused_data_.longitude, dji_position_fused_data_.altitude);
  ros::Time cur_time = ros::Time::now();
  double duration = (cur_time - last_pos_fused_recv_time_).toSec();
  last_mavros_cmd_time_ = cur_time;

  if (duration > 0.5 && position_fused_ready_flag_)
    position_fused_ready_flag_ = false;
  else if (duration <= 0.1 && !position_fused_ready_flag_){
    position_fused_ready_flag_ = true;
    if (dji_ctrl_first_init_){
      dji_ctrl_first_init_ = false;
      T_DjiFlightControllerRidInfo rid_info;
      // rid_info.latitude  = 22.542812;
      // rid_info.longitude = 113.958902;
      // rid_info.altitude  = 10;
      rid_info.latitude  = dji_position_fused_data_.latitude;
      rid_info.longitude = dji_position_fused_data_.longitude;
      rid_info.altitude  = dji_position_fused_data_.altitude;
      djiStat_ = DjiFlightController_Init(rid_info);
      if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        INFO_MSG_RED("[DJI]: init flight controller error, quit program");
        exit(1);
      }
      INFO_MSG_GREEN("\n[DJI]: *** DJI offboard controller first init success !");
    }
  }
}


bool PayloadSdkInterface::djiCreateSubscription(std::string topic_name, E_DjiFcSubscriptionTopic topic,
                                                E_DjiDataSubscriptionTopicFreq frequency,
                                                DjiReceiveDataOfTopicCallback callback){
  djiStat_ = DjiFcSubscription_SubscribeTopic(topic, frequency, callback);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: subscribe topic ["<< topic_name.c_str() << "] error, error code:" << djiStat_);
    return false;
  }
  return true;
}

void PayloadSdkInterface::djiDestroySubscription(std::string topic_name, E_DjiFcSubscriptionTopic topic){
  djiStat_ = DjiFcSubscription_UnSubscribeTopic(topic);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: unsubscribe topic ["<< topic_name.c_str() << "] error, error code:" << djiStat_);
  }
}

template<typename T>
void PayloadSdkInterface::readParam(std::string param_name, T &param_val, T default_val) {
  if (!nh_.param(param_name, param_val, default_val))
    INFO_MSG_YELLOW("[DJI] | param: " << param_name << " not found, using default value: " << default_val);
  else
    INFO_MSG_GREEN("[DJI] | param: " << param_name << " found: " << param_val);
}

bool PayloadSdkInterface::switchCtrlDevice(ctrlDevice device){
  if (device == CTRL_DEVICE_OFFBOARD){
    INFO_MSG_YELLOW("[DJI]: Warning, switch to offboard mode ... ");
    INFO_MSG_YELLOW("[DJI]: Try to get offboard control authority ... ");
    DjiTest_WidgetLogAppend("Try to get offboard control authority");
    djiStat_ = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
      INFO_MSG_RED("[DJI]: obtain joystick control authority error, error code: " << djiStat_);
      DjiTest_WidgetLogAppend("Error: Obtain joystick control authority failed.");
      return false;
    }
    cur_ctrl_device_ = CTRL_DEVICE_OFFBOARD;
    INFO_MSG_GREEN("[DJI]: Obtain joystick control authority success, switch to offboard mode success !");
  }
  else if (device == CTRL_DEVICE_RC){
    INFO_MSG_YELLOW("[DJI]: Warning, switch to rc mode ... ");
    INFO_MSG_YELLOW("[DJI]: Try to stop motion ... ");
    djiStat_ = DjiFlightController_ExecuteEmergencyBrakeAction();
    if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      INFO_MSG_RED("[DJI]: Emergency brake failed, error code: " << djiStat_);
      return false;
    }
    INFO_MSG_YELLOW("[DJI]: do motion recover ... ");
    ros::Duration(2.0).sleep();
    djiStat_ = DjiFlightController_CancelEmergencyBrakeAction();
    if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
      INFO_MSG_RED("Cancel emergency brake action failed, error code: " << djiStat_);
      return false;
    }
    INFO_MSG_YELLOW("[DJI]: Try to release offboard control authority ... ");
    djiStat_ = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
      INFO_MSG_RED("[DJI]: release joystick control authority error, error code: " << djiStat_);
      DjiTest_WidgetLogAppend("Error: Release joystick control authority failed.");
      return false;
    }
    DjiTest_WidgetLogAppend("[DJI]: Switch to rc mode success !");
    dji_flyctrl_pub_timer_.stop();
    cur_ctrl_device_ = CTRL_DEVICE_RC;
  }
  return true;
}
