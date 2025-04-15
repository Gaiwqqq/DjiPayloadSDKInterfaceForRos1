#include "../include/payload_sdk_ros1/payload_sdk_interface.h"

PayloadSdkInterface::PayloadSdkInterface(ros::NodeHandle &nh, T_DjiOsalHandler *osal_handler){
  // ros init
  nh_ = nh;

  // dji node handler init
  dji_osal_handler_ = osal_handler;

  dji_quaternion_data_          = {0};
  dji_velocity_data_            = {0};
  dji_gps_position_data_        = {0};
  dji_timestamp_data_           = {0};
  dji_single_battery_info_data_ = {0};
  dji_position_fused_data_      = {0};

  quaternion_recv_counter_      = 0;
  is_quaternion_disp_           = false;

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
    djiCreateSubscription("quaternion", DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, freq_map[50], nullptr);
  dji_init_success =
    djiCreateSubscription("velocity", DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, freq_map[50], nullptr);
  dji_init_success =
    djiCreateSubscription("gps_position", DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, freq_map[50], nullptr);
  dji_init_success =
    djiCreateSubscription("pos_fusion", DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, freq_map[50], nullptr);

  if (dji_init_success){
    dji_data_read_timer_ = nh_.createTimer(ros::Duration(1.0 / 50.0), &PayloadSdkInterface::djiDataReadCallback, this);
    INFO_MSG_GREEN("[DJI]: Payload SDK init success, do topic init success !");
    INFO_MSG_GREEN("[DJI]: Subscribe to topics: quaternion, velocity, gps_position, pos_fusion");
    INFO_MSG_GREEN("[DJI]: recv frequency : " << 50 << " Hz");
  }
}

PayloadSdkInterface::~PayloadSdkInterface(){

}

void PayloadSdkInterface::djiDataReadCallback(const ros::TimerEvent& event){
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