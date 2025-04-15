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

  quaternion_recv_counter_      = 0;
  is_quaternion_disp_           = false;
  cur_ctrl_mode_                = CTRL_MODE_RC;

  INFO_MSG("[DJI]: Payload SDK init success, do topic init...");

  djiStat_ = DjiFcSubscription_Init();
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    INFO_MSG_RED("[DJI]: init data subscription module error, quit program");
    return;
  }

  T_DjiFlightControllerRidInfo rid_info;
  rid_info.latitude  = 22.542812;
  rid_info.longitude = 113.958902;
  rid_info.altitude  = 10;
  djiStat_ = DjiFlightController_Init(rid_info);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    INFO_MSG_RED("[DJI]: init flight controller error, quit program");
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
    INFO_MSG_GREEN("[DJI]: Payload SDK init success, do topic init success !");
    INFO_MSG_GREEN("[DJI]: Subscribe to topics: quaternion, velocity, gps_position, pos_fusion");
    INFO_MSG_GREEN("[DJI]: recv frequency : " << 50 << " Hz");
  }else{
    INFO_MSG_RED("[DJI]: Payload SDK init failed, do topic init failed !");
    INFO_MSG_RED("[DJI]: Do NOT launch timer! Quit program");
    return ;
  }

  // -------------------- ros init --------------------------//
  mavros_cmd_sub_ = nh_.subscribe("/mavros/set_point/cmd_vel", 2, &PayloadSdkInterface::mavrosCmdCallback,
                                  this , ros::TransportHints().tcpNoDelay());
  offboard_switch_sub_ = nh_.subscribe("/dji/offboard_switch", 2, &PayloadSdkInterface::offboardSwitchCallback,
                                        this, ros::TransportHints().tcpNoDelay());
}

PayloadSdkInterface::~PayloadSdkInterface(){

  INFO_MSG("[DJI]: Payload SDK interface deconstruct");

  djiDestroySubscription("acc_body", DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY);
  djiDestroySubscription("quaternion", DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
  djiDestroySubscription("pos_fusion", DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED);
  djiDestroySubscription("altitude_fused", DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED);
  djiDestroySubscription("velocity", DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
  djiDestroySubscription("gps_position", DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);
  djiDestroySubscription("flight_status", DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT);
  djiDestroySubscription("flight_mode", DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE);
  INFO_MSG("[DJI]: Destoried all subscription topics");

  INFO_MSG("[DJI]: Destory flight controller");

  djiStat_ = DjiFlightController_DeInit();
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                   djiStat_);
  }

  djiStat_ = DjiFcSubscription_DeInit();
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Deinit data subscription module failed, error code:0x%08llX",
                   djiStat_);
  }
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
    quaternion_data_ = Eigen::Vector3d(roll, pitch, yaw);
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
  else
    position_fused_data_ = Eigen::Vector3d(dji_position_fused_data_.latitude, dji_position_fused_data_.longitude, dji_position_fused_data_.altitude);

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

  // flight mdoe
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                 (uint8_t *) &dji_flight_mode_data_,
                                                 sizeof(T_DjiFcSubscriptionDisplaymode),
                                                 &dji_timestamp_data_);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get flight mode data error, timestamp: "
                  << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
}

void PayloadSdkInterface::mavrosCmdCallback(const mavros_msgs::PositionTarget::ConstPtr& msg){

}

void PayloadSdkInterface::offboardSwitchCallback(const std_msgs::Int8::ConstPtr& msg){
  std_msgs::Int8 mode = *msg;
  if (mode.data == 1 && cur_ctrl_mode_ == CTRL_MODE_RC){
    switchCtrlMode(CTRL_MODE_OFFBOARD);
  }
  else if (mode.data == 0 && cur_ctrl_mode_ == CTRL_MODE_OFFBOARD){
    switchCtrlMode(CTRL_MODE_RC);
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

bool PayloadSdkInterface::switchCtrlMode(ctrlMode mode){
  if (mode == CTRL_MODE_OFFBOARD){
    INFO_MSG_YELLOW("[DJI]: Warning, switch to offboard mode ... ");
    INFO_MSG_YELLOW("[DJI]: Try to get offboard control authority ... ");
    DjiTest_WidgetLogAppend("Try to get offboard control authority");
    djiStat_ = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
      INFO_MSG_RED("[DJI]: obtain joystick control authority error, error code: " << djiStat_);
      DjiTest_WidgetLogAppend("Error: Obtain joystick control authority failed.");
      return false;
    }
    cur_ctrl_mode_ = CTRL_MODE_OFFBOARD;
    INFO_MSG_GREEN("[DJI]: Obtain joystick control authority success, switch to offboard mode success !");
  }
  else if (mode == CTRL_MODE_RC){
    INFO_MSG_YELLOW("[DJI]: Warning, switch to rc mode ... ");
    INFO_MSG_YELLOW("[DJI]: Try to release offboard control authority ... ");
    DjiTest_WidgetLogAppend("Try to release offboard control authority");
    djiStat_ = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
      INFO_MSG_RED("[DJI]: release joystick control authority error, error code: " << djiStat_);
      DjiTest_WidgetLogAppend("Error: Release joystick control authority failed.");
      return false;
    }
    cur_ctrl_mode_ = CTRL_MODE_RC;
  }
  return true;
}
