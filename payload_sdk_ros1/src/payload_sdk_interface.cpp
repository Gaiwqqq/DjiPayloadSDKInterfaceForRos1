#include "../include/payload_sdk_ros1/payload_sdk_interface.h"

PayloadSdkInterface::PayloadSdkInterface(ros::NodeHandle &nh, T_DjiOsalHandler *osal_handler){
  // ros init
  nh_ = nh;

  // -------------------- DJI init --------------------------//
  dji_osal_handler_ = osal_handler;

  quaternion_recv_counter_      = 0;
  is_quaternion_disp_           = false;
  is_gps_convergent_            = false;
  cur_ctrl_device_              = CTRL_DEVICE_RC;
  cytl_cmd_heartbeat_ready_     = false;
  position_fused_ready_flag_    = false;
  dji_ctrl_first_init_          = true;
  gps_ready_                    = false;

  last_ctrl_cmd_time_     = ros::Time::now();
  last_pos_fused_recv_time_ = last_ctrl_cmd_time_;

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.0;
  path_vis_data_.poses.push_back(pose);

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
  freq_map[400] = DJI_DATA_SUBSCRIPTION_TOPIC_400_HZ;
  freq_map[200] = DJI_DATA_SUBSCRIPTION_TOPIC_200_HZ;
  freq_map[100] = DJI_DATA_SUBSCRIPTION_TOPIC_100_HZ;
  freq_map[50]  = DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ;
  freq_map[10]  = DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ;
  freq_map[5]   = DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ;
  freq_map[1]   = DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ;

  bool dji_init_success;
  dji_init_success =
          djiCreateSubscription("acc_body", DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY, freq_map[10], nullptr);
  dji_init_success =
          djiCreateSubscription("acc_ground", DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND, freq_map[10], nullptr);
  dji_init_success =
          djiCreateSubscription("acc_raw", DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW, freq_map[50], nullptr);
  dji_init_success =
          djiCreateSubscription("angular_rate_fused", DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED, freq_map[50], nullptr);
  dji_init_success =
          djiCreateSubscription("quaternion", DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, freq_map[50], nullptr);
  dji_init_success =
          djiCreateSubscription("pos_fusion", DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, freq_map[50], nullptr);
  dji_init_success =
          djiCreateSubscription("altitude_fused", DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED, freq_map[50], nullptr);
  dji_init_success =
          djiCreateSubscription("velocity", DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, freq_map[50], nullptr);
  dji_init_success =
          djiCreateSubscription("gps_position", DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION, freq_map[5], nullptr);
  dji_init_success =
          djiCreateSubscription("gps_details", DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS, freq_map[5], nullptr);
  dji_init_success =
          djiCreateSubscription("flight_status", DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, freq_map[5], nullptr);
  dji_init_success =
          djiCreateSubscription("flight_mode", DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, freq_map[5], nullptr);
  dji_init_success =
          djiCreateSubscription("ctrl_device", DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE, freq_map[1], nullptr);
  dji_init_success =
          djiCreateSubscription("rtk_state", DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS, freq_map[5], nullptr);
  dji_init_success =
          djiCreateSubscription("rtk_pos", DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION, freq_map[5], nullptr);
  dji_init_success =
          djiCreateSubscription("rtk_vel", DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY, freq_map[5], nullptr);
  dji_init_success =
          djiCreateSubscription("rtk_yaw", DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW, freq_map[5], nullptr);
  dji_init_success =
          djiCreateSubscription("rc", DJI_FC_SUBSCRIPTION_TOPIC_RC, freq_map[10], nullptr);

  // -------------------- ros init --------------------------//
  std::string topic_nav_pub, topic_ctrl_sub, topic_livox_sub;
  bool livox_trans_enable;
  readParam<std::string>("dji/topic_nav_pub", topic_nav_pub, "/mavros/nav_msgs");
  readParam<std::string>("dji/topic_ctrl_sub", topic_ctrl_sub, "/mavros/setpoint_raw/local");
  readParam<double>("dji/gps_accuracy_threshold", _gps_accuracy_thres, 2.0);
  readParam<double>("dji/data_loop_rate", _data_loop_rate, 10.0);
  readParam<string>("dji/cmd_type", ctrl_cmd_type_, "mavros");
  readParam<string>("dji/livox_sub_topic", topic_livox_sub, "/livox/lidar");
  readParam<bool>("dji/livox_trans_enable", livox_trans_enable, false);


  if (ctrl_cmd_type_ == "mavros")
    ctrl_cmd_sub_ = nh_.subscribe(topic_ctrl_sub, 2, &PayloadSdkInterface::mavrosCmdCallback,
                                  this , ros::TransportHints().tcpNoDelay());
  else if (ctrl_cmd_type_ == "60")
    custom_60_cmd_sub_ = nh_.subscribe(topic_ctrl_sub, 2, &PayloadSdkInterface::custom60CmdCallback,
                                       this, ros::TransportHints().tcpNoDelay());

  offboard_switch_sub_ = nh_.subscribe("/dji/offboard_switch", 2, &PayloadSdkInterface::offboardSwitchCallback,
                                       this, ros::TransportHints().tcpNoDelay());
  imu_60_pub_          = nh_.advertise<com_package::imu_60>(topic_nav_pub, 2);
  odom_trans_pub_      = nh_.advertise<nav_msgs::Odometry>("/dji/odom_trans", 2);
  imu_trans_pub_       = nh_.advertise<sensor_msgs::Imu>("/dji/imu_trans", 2);
  vis_pub_             = nh_.advertise<visualization_msgs::Marker>("/dji/vis", 2);
  path_vis_pub_        = nh_.advertise<nav_msgs::Path>("/dji/path_vis", 2);

  if (livox_trans_enable){
    livoxTransInit();
    livox_sub_         = nh_.subscribe(topic_livox_sub, 2, &PayloadSdkInterface::livoxCallback,
                                       this, ros::TransportHints().tcpNoDelay());
    livox_pub_         = nh_.advertise<sensor_msgs::PointCloud2>("/dji/livox", 2);
  }

  if (dji_init_success){
    dji_data_read_timer_   = nh_.createTimer(ros::Duration(1.0 / _data_loop_rate), &PayloadSdkInterface::djiDataReadCallback, this);
    dji_flyctrl_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / 50.0), &PayloadSdkInterface::djiFlyCtrlPubCallback, this);
    dji_flyctrl_pub_timer_.stop();
    INFO_MSG_GREEN("[DJI] | Payload SDK init success, do topic init success !");
    INFO_MSG_GREEN("[DJI] | Subscribe to topics: quaternion, velocity, gps_position, pos_fusion");
    INFO_MSG_GREEN("[DJI] | data publish max frequency : " << 50 << " Hz");
  }else{
    INFO_MSG_RED("[DJI] | Payload SDK init failed, do topic init failed !");
    INFO_MSG_RED("[DJI] | Do NOT launch timer! Quit program");
    return ;
  }
}

PayloadSdkInterface::~PayloadSdkInterface(){

  INFO_MSG("[DJI]: Payload SDK interface deconstruct");
  INFO_MSG("[DJI]: Destory flight controller");

  if (cur_ctrl_device_ == CTRL_DEVICE_OFFBOARD)
    switchCtrlDevice(CTRL_DEVICE_RC);

  djiStat_ = DjiFlightController_DeInit();
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX", djiStat_);
  }

  djiDestroySubscription("acc_body", DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY);
  djiDestroySubscription("acc_ground", DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND);
  djiDestroySubscription("acc_raw", DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW);
  djiDestroySubscription("angular_rate_fused", DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED);
  djiDestroySubscription("quaternion", DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION);
  djiDestroySubscription("pos_fusion", DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED);
  djiDestroySubscription("altitude_fused", DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED);
  djiDestroySubscription("velocity", DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
  djiDestroySubscription("gps_position", DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);
  djiDestroySubscription("gps_details", DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS);
  djiDestroySubscription("flight_status", DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT);
  djiDestroySubscription("flight_mode", DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE);
  djiDestroySubscription("ctrl_device", DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE);
  djiDestroySubscription("rtk_state", DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS);
  djiDestroySubscription("rtk_pos", DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION);
  djiDestroySubscription("rtk_vel", DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY);
  djiDestroySubscription("rtk_yaw", DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW);
  djiDestroySubscription("rc", DJI_FC_SUBSCRIPTION_TOPIC_RC);
  INFO_MSG("[DJI]: Destoried all subscription topics");

  djiStat_ = DjiFcSubscription_DeInit();
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Deinit data subscription module failed, error code:0x%08llX", djiStat_);
  }
  INFO_MSG("[DJI]: Destoried data subscription module");
  INFO_MSG("[DJI]: Payload SDK interface deconstruct success... Goodbye!");
}

void PayloadSdkInterface::livoxTransInit() {
  double x, y, z, pitch, roll, yaw;
  readParam<double>("dji/livox_trans_matrix/x", x, 0.0);
  readParam<double>("dji/livox_trans_matrix/y", y, 0.0);
  readParam<double>("dji/livox_trans_matrix/z", z, 0.0);
  readParam<double>("dji/livox_trans_matrix/pitch", pitch, 0.0);
  readParam<double>("dji/livox_trans_matrix/roll", roll, 0.0);
  readParam<double>("dji/livox_trans_matrix/yaw", yaw, 0.0);

  pitch = pitch * M_PI / 180.0;
  roll = roll * M_PI / 180.0;
  yaw = yaw * M_PI / 180.0;

  Eigen::Matrix3d rotX;
  rotX << 1, 0, 0,
          0, cos(roll), -sin(roll),
          0, sin(roll), cos(roll);

  Eigen::Matrix3d rotY;
  rotY << cos(pitch), 0, sin(pitch),
          0, 1, 0,
          -sin(pitch), 0, cos(pitch);

  Eigen::Matrix3d rotZ;
  rotZ << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw), 0,
          0, 0, 1;

  Eigen::Matrix3d rotation = rotZ * rotY * rotX;
  _livox2body_matrix = Eigen::Matrix4d::Identity();
  _livox2body_matrix.block<3, 3>(0, 0) = rotation;
  _livox2body_matrix(0, 3) = x;
  _livox2body_matrix(1, 3) = y;
  _livox2body_matrix(2, 3) = z;
}

void PayloadSdkInterface::djiDataReadCallback(const ros::TimerEvent& event){
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

  // Acc body
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_BODY,
                                                     (uint8_t *) &dji_acc_body_data_,
                                                     sizeof(T_DjiFcSubscriptionAccelerationBody),
                                                     &dji_timestamp_data_);

  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get acc body data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else{
    acc_body_data_ = Eigen::Vector3d(dji_acc_body_data_.x, dji_acc_body_data_.y, dji_acc_body_data_.z);
  }

  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_GROUND,
                                                     (uint8_t *) &dji_acc_ground_data_,
                                                     sizeof(T_DjiFcSubscriptionAccelerationGround),
                                                     &dji_timestamp_data_);
  if (djiStat_!= DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get acc ground data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }else {
    acc_ground_data_ = Eigen::Vector3d(dji_acc_ground_data_.x, dji_acc_ground_data_.y, dji_acc_ground_data_.z);
//    std::cout << "acc_ground_data_: " << acc_ground_data_.transpose() << std::endl;
  }

  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW,
                                                     (uint8_t *) &dji_acc_raw_data_,
                                                     sizeof(T_DjiFcSubscriptionAccelerationRaw),
                                                     &dji_timestamp_data_);
  if (djiStat_!= DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get acc raw data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }else {
    acc_raw_data_ = Eigen::Vector3d(dji_acc_raw_data_.x, dji_acc_raw_data_.y, dji_acc_raw_data_.z);
    // std::cout << "acc_raw_data_: " << acc_raw_data_.transpose() << std::endl;
  }

  // angular rate fused
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ANGULAR_RATE_FUSIONED,
                                                     (uint8_t *) &dji_angular_rate_fused_data_,
                                                     sizeof(T_DjiFcSubscriptionAngularRateFusioned),
                                                     &dji_timestamp_data_);

  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get angular rate fused data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else{
    angular_rate_fused_data_ = Eigen::Vector3d(dji_angular_rate_fused_data_.x, dji_angular_rate_fused_data_.y, dji_angular_rate_fused_data_.z);
//    std::cout << "angular_rate_fused_data_: " << angular_rate_fused_data_.transpose() << std::endl;
  }

  // Vel
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                     (uint8_t *) &dji_velocity_data_,
                                                     sizeof(T_DjiFcSubscriptionVelocity),
                                                     &dji_timestamp_data_);

  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get velocity data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else feedVelDataProcess();

  // Quaternion
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                     (uint8_t *) &dji_quaternion_data_,
                                                     sizeof(T_DjiFcSubscriptionQuaternion),
                                                     &dji_timestamp_data_);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get quaternion data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else feedQuaternionDataProcess();

  // GPS position
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
                                                     (uint8_t *) &dji_gps_position_data_,
                                                     sizeof(T_DjiFcSubscriptionGpsPosition),
                                                     &dji_timestamp_data_);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get gps position data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else{
    gps_position_data_ = Eigen::Vector3d(dji_gps_position_data_.x, dji_gps_position_data_.y, dji_gps_position_data_.z);
  }

  // GPS details
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
                                                     (uint8_t *) &dji_gps_details_data_,
                                                     sizeof(T_DjiFcSubscriptionGpsDetails),
                                                     &dji_timestamp_data_);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get gps details data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }else{
    feedGPSDetailsDataProcess();
  }

  // altitude fused
  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                     (uint8_t *) &dji_altitude_fused_data_,
                                                     sizeof(T_DjiFcSubscriptionAltitudeFused),
                                                     &dji_timestamp_data_);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
    INFO_MSG_RED("[DJI]: get altitude fused data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }
  else{
    altitude_fused_data_ = dji_altitude_fused_data_;
  }

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

  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE,
                                                     (uint8_t *) &dji_ctrl_device_data_,
                                                     sizeof(T_DjiFcSubscriptionControlDevice),
                                                     &dji_timestamp_data_);
  if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    INFO_MSG_RED("[DJI]: get control device data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }

  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_CONNECT_STATUS,
                                                     (uint8_t *) &dji_rtk_connection_stat_data_,
                                                     sizeof(T_DjiFcSubscriptionRTKConnectStatus),
                                                     &dji_timestamp_data_);
  if (djiStat_!= DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    INFO_MSG_RED("[DJI]: get rtk state data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }

  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
                                                     (uint8_t *) &dji_rtk_pos_data_,
                                                     sizeof(T_DjiFcSubscriptionRtkPosition),
                                                     &dji_timestamp_data_);
  if (djiStat_!= DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    INFO_MSG_RED("[DJI]: get rtk pos data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }

  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_VELOCITY,
                                                     (uint8_t *) &dji_rtk_vel_data_,
                                                     sizeof(T_DjiFcSubscriptionRtkVelocity),
                                                     &dji_timestamp_data_);
  if (djiStat_!= DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    INFO_MSG_RED("[DJI]: get rtk vel data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }

  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_YAW,
                                                     (uint8_t *) &dji_rtk_yaw_data_,
                                                     sizeof(T_DjiFcSubscriptionRtkYaw),
                                                     &dji_timestamp_data_);
  if (djiStat_!= DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    INFO_MSG_RED("[DJI]: get rtk yaw data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }

  djiStat_ = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC,
                                                     (uint8_t *) &dji_rc_data_,
                                                     sizeof(T_DjiFcSubscriptionRC),
                                                     &dji_timestamp_data_);
  if (djiStat_!= DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    INFO_MSG_RED("[DJI]: get rc data error, timestamp: "
                         << dji_timestamp_data_.microsecond << " ms, error code: " << djiStat_);
  }else{
//    INFO_MSG("***rc data roll :" << dji_rc_data_.roll << " pitch : " << dji_rc_data_.pitch << " yaw : " << dji_rc_data_.yaw << " throttle : " << dji_rc_data_.throttle);
//    INFO_MSG("rc data mode : " << dji_rc_data_.mode << " gear : " << dji_rc_data_.gear);
  }

  // ----------------------- ROS publish -----------------------------//
  ros::Time cur_t = ros::Time::now();
  publishImu60Data();
  publishOdomData();
  publishImuMavrosData();
  drawVel();
  drawRangeCircles();
  drawPath();
  ROS_INFO_STREAM_THROTTLE(2.0, "[DJI]: Main data recv process spend time : " <<
                            (ros::Time::now() - cur_t).toSec() * 1e3 << " ms");
}

// 机体坐标系 FRD (前右下)
// 大地坐标系 NED (北东地)
void PayloadSdkInterface::djiFlyCtrlPubCallback(const ros::TimerEvent& event){
  ros::Time cur_time = ros::Time::now();
  double time_duration = (cur_time - last_ctrl_cmd_time_).toSec();
  if (time_duration > 0.5 && cytl_cmd_heartbeat_ready_){
    INFO_MSG_RED("\n ***[DJI]: Warning, ctrl cmd data not received in 500ms, lost heartbeat !");
    cytl_cmd_heartbeat_ready_ = false;
    vel_ctrl_cmd_data_frd_ = Eigen::Vector4d::Zero();
  }
  else if (time_duration < 0.5 && !cytl_cmd_heartbeat_ready_){
    INFO_MSG_GREEN("\n ***[DJI]: ctrl cmd data heartbeat recovered !");
    cytl_cmd_heartbeat_ready_ = true;
  }

  if (!cytl_cmd_heartbeat_ready_) return;
  if (cur_ctrl_device_ != CTRL_DEVICE_OFFBOARD) return;
  if (cur_ctrl_mode_ == NOT_SET) return ;

  if (cur_ctrl_mode_ == OFFBOARD_VEL_BODY){
    if (ctrl_cmd_type_ == "mavros"){
      if (mavros_cmd_data_recv_.type_mask != mavros_cmd_type_mask_velctrl_only_ ||
          mavros_cmd_data_recv_.coordinate_frame != mavros_msgs::PositionTarget::FRAME_BODY_NED){
        INFO_MSG_RED("[DJI]: Warning, mavros cmd data type mask not match, only vel-body-ctrl data is accepted !");
        return;
      }
    }
    T_DjiFlightControllerJoystickCommand joystick_cmd =
            {static_cast<dji_f32_t>(vel_ctrl_cmd_data_frd_.x()),
             static_cast<dji_f32_t>(vel_ctrl_cmd_data_frd_.y()),
             static_cast<dji_f32_t>(vel_ctrl_cmd_data_frd_.z()),
             static_cast<dji_f32_t>(vel_ctrl_cmd_data_frd_.w())};
    DjiFlightController_ExecuteJoystickAction(joystick_cmd);
  }
}

void PayloadSdkInterface::livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
  if (msg->points.empty()) return;

  Eigen::Matrix4d livox2body_matrix = _livox2body_matrix;

  ros::Time cur_time = ros::Time::now();
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  for (auto &point : msg->points) {
    Eigen::Vector4d point_4d(point.x, point.y, point.z, 1.0);
    Eigen::Vector4d point_4d_trans = livox2body_matrix * point_4d;

    pcl::PointXYZI pcl_point;
    pcl_point.x = point_4d_trans.x();
    pcl_point.y = point_4d_trans.y();
    pcl_point.z = point_4d_trans.z();
    pcl_point.intensity = point.reflectivity;
    pcl_cloud.push_back(pcl_point);
  }
  pcl::toROSMsg(pcl_cloud, cloud_msg);
  cloud_msg.header.frame_id = "livox_frame";
  cloud_msg.header.stamp = ros::Time::now();
  livox_pub_.publish(cloud_msg);

  ROS_WARN_STREAM_THROTTLE(2.0, "[DJI]: Livox data recv process spend time : "
            << (ros::Time::now() - cur_time).toSec() * 1e3 << " ms");
}

void PayloadSdkInterface::mavrosCmdCallback(const mavros_msgs::PositionTarget::ConstPtr& msg){
  last_pos_fused_recv_time_ = ros::Time::now();
  mavros_cmd_data_recv_ = *msg;
  vel_ctrl_cmd_data_frd_[0] =  msg->velocity.x;
  vel_ctrl_cmd_data_frd_[1] = -msg->velocity.y;
  vel_ctrl_cmd_data_frd_[2] = -msg->velocity.z;
  vel_ctrl_cmd_data_frd_[3] =  msg->yaw_rate / 180.0 * M_PI;
}

void PayloadSdkInterface::custom60CmdCallback(const flyctrl::flyctrl_send::ConstPtr &msg) {
  last_ctrl_cmd_time_ = ros::Time::now();
  custom_60_cmd_data_recv_ = *msg;
  vel_ctrl_cmd_data_frd_[0] =  msg->u_sp;
  vel_ctrl_cmd_data_frd_[1] =  msg->v_sp;
  vel_ctrl_cmd_data_frd_[2] = -msg->w_sp;
  vel_ctrl_cmd_data_frd_[3] =  msg->r_sp / 180.0 * M_PI;
}

void PayloadSdkInterface::offboardSwitchCallback(const std_msgs::Int8::ConstPtr& msg){
  std_msgs::Int8 mode = *msg;
  if (!gps_ready_ || !position_fused_ready_flag_ || !cytl_cmd_heartbeat_ready_){
    INFO_MSG_RED("[DJI]: Not ready, Can't switch control device !");
    return ;
  }
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
  com_package::imu_60 imu60_msg;
  ros::WallTime t = ros::WallTime::now();
  imu60_msg.header.stamp = ros::Time(t.sec, t.nsec);

  imu60_msg.lat       = static_cast<float>(position_fused_data_.x());
  imu60_msg.lon       = static_cast<float>(position_fused_data_.y());
  imu60_msg.alt       = static_cast<float>(position_fused_data_.z());
  imu60_msg.Vx        = static_cast<float>(velocity_data_neu_.x());
  imu60_msg.Vy        = static_cast<float>(velocity_data_neu_.y());
  imu60_msg.Vz        = static_cast<float>(velocity_data_neu_.z());

  double yaw_rad = quaternion_data_.z() / 180.0 * M_PI;
  yaw_rad = M_PI / 2.0 - yaw_rad;
  if (yaw_rad < 0) yaw_rad = M_PI * 2.0 + yaw_rad;
  double yaw_fix = yaw_rad * 180 / M_PI;

  imu60_msg.Pitch     = static_cast<float>(quaternion_data_.x());
  imu60_msg.Roll      = static_cast<float>(quaternion_data_.y());
  imu60_msg.Yaw       = static_cast<float>(yaw_fix);

  imu60_msg.RollRate  = static_cast<float>(angular_rate_fused_data_.x());
  imu60_msg.PitchRate = static_cast<float>(angular_rate_fused_data_.y());
  imu60_msg.YawRate   = static_cast<float>(angular_rate_fused_data_.z());

  imu60_msg.Ax        = static_cast<float>(acc_raw_data_.x());
  imu60_msg.Ay        = static_cast<float>(-acc_raw_data_.y());
  imu60_msg.Az        = static_cast<float>(-acc_raw_data_.z());

  imu60_msg.SensorStatus = 25;
  imu60_msg.WorkStatus   = 8;
  imu60_msg.NaviStatus   = 9;

  imu_60_pub_.publish(imu60_msg);
}

void PayloadSdkInterface::publishOdomData(){
  if (!gps_ready_) return ;
//  nav_msgs::Odometry odom;
//  odom.header.stamp            = ros::Time::now();
//  odom.header.frame_id         = "world";
//  odom.pose.pose.position.x    = xyz_pos_neu_.x();
//  odom.pose.pose.position.y    = xyz_pos_neu_.y();
//  odom.pose.pose.position.z    = xyz_pos_neu_.z();
//
//  odom.pose.pose.orientation.x = quaternion_world_.x();
//  odom.pose.pose.orientation.y = quaternion_world_.y();
//  odom.pose.pose.orientation.z = quaternion_world_.z();
//  odom.pose.pose.orientation.w = quaternion_world_.w();
//
//  odom.twist.twist.linear.x    = velocity_data_frd_.x();
//  odom.twist.twist.linear.y    = velocity_data_frd_.y();
//  odom.twist.twist.linear.z    = velocity_data_frd_.z();
//  odom.twist.twist.angular.x   = angular_rate_fused_data_.x();
//  odom.twist.twist.angular.y   = angular_rate_fused_data_.y();
//  odom.twist.twist.angular.z   = angular_rate_fused_data_.z();
//
//  odom_trans_pub_.publish(odom);

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = ros::Time::now();
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id  = "dji_body";
  tf_msg.transform.rotation.x = quaternion_world_.x();
  tf_msg.transform.rotation.y = quaternion_world_.y();
  tf_msg.transform.rotation.z = quaternion_world_.z();
  tf_msg.transform.rotation.w = quaternion_world_.w();

  tf_msg.transform.translation.x = xyz_pos_neu_.x();
  tf_msg.transform.translation.y = xyz_pos_neu_.y();
  tf_msg.transform.translation.z = xyz_pos_neu_.z();
  tf_broadcaster_.sendTransform(tf_msg);
}

void PayloadSdkInterface::publishImuMavrosData(){
  if (!gps_ready_) return ;
  sensor_msgs::Imu imu_msg;

  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = "world";

  imu_msg.orientation.x = quaternion_world_.x();
  imu_msg.orientation.y = quaternion_world_.y();
  imu_msg.orientation.z = quaternion_world_.z();
  imu_msg.orientation.w = quaternion_world_.w();

  imu_msg.linear_acceleration.x =  acc_raw_data_.x();
  imu_msg.linear_acceleration.y = -acc_raw_data_.y();
  imu_msg.linear_acceleration.z = -acc_raw_data_.z();

  imu_msg.angular_velocity.x    = angular_rate_fused_data_.x();
  imu_msg.angular_velocity.y    = angular_rate_fused_data_.y();
  imu_msg.angular_velocity.z    = angular_rate_fused_data_.z();
  imu_trans_pub_.publish(imu_msg);
}


void PayloadSdkInterface::feedPositionDataProcess(){
  position_fused_data_ =
          Eigen::Vector3d(dji_position_fused_data_.latitude  / M_PI * 180.0,   // 纬
                          dji_position_fused_data_.longitude / M_PI * 180.0,   // 经
                          dji_position_fused_data_.altitude);
  if (gps_ready_){
    xyz_pos_neu_ = LLA2XYZ(position_fused_data_);
//    std::cout << "xyz_pos_neu_: " << xyz_pos_neu_.transpose() << std::endl;
    // std::cout << "positon fused data: " << position_fused_data_.transpose() << std::endl;
  }

  ros::Time cur_time = ros::Time::now();
  double duration = (cur_time - last_pos_fused_recv_time_).toSec();
  last_pos_fused_recv_time_ = cur_time;

  if (duration > 0.5 && position_fused_ready_flag_)
    position_fused_ready_flag_ = false;
  else if (duration <= 0.11 && !position_fused_ready_flag_){
    position_fused_ready_flag_ = true;
    if (dji_ctrl_first_init_ && gps_ready_){
      INFO_MSG_YELLOW("\n\n[DJI]: *** DJI offboard controller first init ...");
      dji_ctrl_first_init_ = false;
      T_DjiFlightControllerRidInfo rid_info;
      // rid_info.latitude  = 22.542812;
      // rid_info.longitude = 113.958902;
      // rid_info.altitude  = 10;
      rid_info.latitude  = dji_position_fused_data_.latitude;
      rid_info.longitude = dji_position_fused_data_.longitude;
      rid_info.altitude  = static_cast<uint16_t> (dji_position_fused_data_.altitude);
      djiStat_ = DjiFlightController_Init(rid_info);
      if (djiStat_ != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        INFO_MSG_RED("[DJI]: init flight controller error, quit program");
        exit(1);
      }
      INFO_MSG_GREEN("[DJI]: *** DJI offboard controller first init success !\n\n");
    }
  }
}

void PayloadSdkInterface::feedGPSDetailsDataProcess(){
  gps_pos_accuracy_ = dji_gps_details_data_.pdop;
  if (gps_pos_accuracy_ <= _gps_accuracy_thres && !gps_ready_){
    gps_ready_ = true;
    neu_pos_init_ = position_fused_data_;
    INFO_MSG_GREEN("[DJI]: GPS position accuracy is good, ready to fly !");
    INFO_MSG_GREEN("[DJI]: POSE FUSED DATA INIT -> " << position_fused_data_.transpose());
    INFO_MSG_GREEN("[DJI]: GPS INIT position: " << position_fused_data_.transpose());
  }
}

void PayloadSdkInterface::feedVelDataProcess() {
  // 已知无人机姿态，将速度从北东天坐标系转换为机体FLU坐标系
  velocity_data_neu_     = Eigen::Vector3d (dji_velocity_data_.data.x, dji_velocity_data_.data.y, dji_velocity_data_.data.z);
  velocity_data_neu_vis_ = Eigen::Vector3d (dji_velocity_data_.data.x, -dji_velocity_data_.data.y, dji_velocity_data_.data.z);
  velocity_data_frd_     = quaternion_world_ * velocity_data_neu_;
  velocity_data_frd_vis_ = quaternion_world_ * velocity_data_neu_vis_;
}

void PayloadSdkInterface::feedQuaternionDataProcess() {
  T_DjiFcSubscriptionQuaternion quaternion = dji_quaternion_data_;
  double pitch = (dji_f64_t) asinf(-2 * quaternion.q1 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q2) * 57.3;
  double roll = (dji_f64_t) atan2f(2 * quaternion.q2 * quaternion.q3 + 2 * quaternion.q0 * quaternion.q1,
                                   -2 * quaternion.q1 * quaternion.q1 - 2 * quaternion.q2 * quaternion.q2 + 1) * 57.3;
  double yaw = (dji_f64_t) atan2f(2 * quaternion.q1 * quaternion.q2 + 2 * quaternion.q0 * quaternion.q3,
                                  -2 * quaternion.q2 * quaternion.q2 - 2 * quaternion.q3 * quaternion.q3 + 1) * 57.3;
  quaternion_data_ = Eigen::Vector3d(pitch, roll, yaw);
//  std::cout << "(pitch, roll, yaw): " << quaternion_data_.transpose() << std::endl;
  Eigen::Quaterniond dji_quat(
          dji_quaternion_data_.q0,
          dji_quaternion_data_.q1,
          dji_quaternion_data_.q2,
          dji_quaternion_data_.q3
  );
  quaternion_world_ = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) * dji_quat;
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

/**
 * @brief 从 ROS 参数服务器读取参数值。
 *
 * 该函数是一个模板函数，可用于读取不同类型的参数。如果在 ROS 参数服务器中找到指定名称的参数，
 * 则将其值赋给传入的引用变量；若未找到，则使用默认值，并输出相应的提示信息。
 *
 * @tparam T 参数的类型，函数会根据实际传入的参数类型自动推导。
 * @param param_name 要读取的参数在 ROS 参数服务器中的名称。
 * @param param_val 用于存储读取到的参数值的引用变量。
 * @param default_val 当参数未在 ROS 参数服务器中找到时使用的默认值。
 */
template<typename T>
void PayloadSdkInterface::readParam(std::string param_name, T &param_val, T default_val) {
  if (!nh_.param(param_name, param_val, default_val))
    INFO_MSG_YELLOW("[DJI] | param: " << param_name << " not found, using default value: " << default_val);
  else
    INFO_MSG_GREEN("[DJI] | param: " << param_name << " found: " << param_val);
}

/**
 * @brief 切换飞行器的控制设备。
 *
 * 该函数可将飞行器的控制设备在机载控制（offboard）模式和遥控器（RC）模式之间进行切换。
 * 若切换到机载控制模式，会尝试获取机载控制权限；若切换到遥控器模式，会先执行紧急制动，
 * 随后恢复运动并释放机载控制权限。
 *
 * @param device 目标控制设备，取值为 `CTRL_DEVICE_OFFBOARD` 或 `CTRL_DEVICE_RC`。
 * @return bool 若切换成功返回 `true`，否则返回 `false`。
 */
bool PayloadSdkInterface::switchCtrlDevice(CtrlDevice device){
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

// 原始速度是东北地坐标系下的速度 NED
void PayloadSdkInterface::drawVel() {
  // 绘制箭头，长度为速度大小，方向为速度方向
  visualization_msgs::Marker marker;
  marker.header.frame_id = "dji_body";
  marker.header.stamp    = ros::Time::now();
  marker.ns     = "velocity";
  marker.id     = 0;
  marker.type   = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.points.resize(2);
  marker.points[0].x = 0.0;
  marker.points[0].y = 0.0;
  marker.points[0].z = 0.0;
  marker.points[1].x = velocity_data_frd_vis_.x() * 5;
  marker.points[1].y = velocity_data_frd_vis_.y() * 5;
  marker.points[1].z = velocity_data_frd_vis_.z() * 5;

  marker.pose.orientation.w = 1.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  vis_pub_.publish(marker);
}

void PayloadSdkInterface::drawRangeCircles() {
  std::vector<double> distances = {10.0, 30.0, 50.0}; // 定义要可视化的距离范围
  std::vector<std_msgs::ColorRGBA> colors;
  // 定义不同圆对应的颜色
  std_msgs::ColorRGBA color1, color2, color3;
  color1.r = 1.0; color1.g = 0.0; color1.b = 0.0; color1.a = 0.3; // 红色
  color2.r = 0.0; color2.g = 1.0; color2.b = 0.0; color2.a = 0.3; // 绿色
  color3.r = 0.0; color3.g = 0.0; color3.b = 1.0; color3.a = 0.3; // 蓝色
  colors = {color1, color2, color3};

  for (size_t i = 0; i < distances.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world"; // 设置参考坐标系
    marker.header.stamp    = ros::Time::now();
    marker.ns              = "range_circles";
    marker.id              = static_cast<int>(i);
    marker.type            = visualization_msgs::Marker::CYLINDER; // 使用圆柱来近似圆
    marker.action          = visualization_msgs::Marker::ADD;

    // 设置位置，假设圆中心在原点，可按需修改
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0 - static_cast<double>(i) * 0.1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // 设置尺寸，高度设小以模拟圆
    marker.scale.x = distances[i] * 2;
    marker.scale.y = distances[i] * 2;
    marker.scale.z = 0.01;

    // 设置颜色
    marker.color = colors[i];

    vis_pub_.publish(marker);
  }
}

void PayloadSdkInterface::drawPath() {
  path_vis_data_.header.stamp = ros::Time::now();
  path_vis_data_.header.frame_id = "world";
  geometry_msgs::PoseStamped last_pos = path_vis_data_.poses.back();
  if ((xyz_pos_neu_ - Eigen::Vector3d(last_pos.pose.position.x,
                                      last_pos.pose.position.y,
                                      last_pos.pose.position.z)).norm() > 0.15){
    geometry_msgs::PoseStamped new_pos;
    new_pos.header.frame_id = "world";
    new_pos.header.stamp = path_vis_data_.header.stamp;
    new_pos.pose.position.x = xyz_pos_neu_.x();
    new_pos.pose.position.y = xyz_pos_neu_.y();
    new_pos.pose.position.z = xyz_pos_neu_.z();
    path_vis_data_.poses.push_back(new_pos);
  }
  path_vis_pub_.publish(path_vis_data_);
}

Eigen::Vector3d PayloadSdkInterface::XYZ2LLA(const Eigen::Vector3d& xyz){
  double Ax = 6383487.606;
  double Bx = 5357.31;
  double Ay = 6367449.134;
  double By = 32077.0;
  double dPI = 57.295779512; // 角度转弧度的系数（180/π）

  // 初始猜测平均纬度，这里简单用初始纬度
  double lat_a = neu_pos_init_.x();
  double tolerance = 1e-8;
  double delta = 1.0;

  // 迭代求解平均纬度
  while (delta > tolerance) {
    double term1 = Ay - By * std::pow(std::cos(lat_a / dPI), 2);
    double new_lat = (xyz.x() * dPI / term1) + neu_pos_init_.x();
    double new_lat_a = (neu_pos_init_.x() + new_lat) / 2.0;
    delta = std::abs(new_lat_a - lat_a);
    lat_a = new_lat_a;
  }

  double lat = (xyz.x() * dPI) / (Ay - By * std::pow(std::cos(lat_a / dPI), 2)) + neu_pos_init_.x();
  double lon = - (xyz.y() * dPI) / (Ax * std::cos(lat_a / dPI) - Bx * std::pow(std::cos(lat_a / dPI), 3)) + neu_pos_init_.y();
  double alt = xyz.z() + neu_pos_init_.z();

  return Eigen::Vector3d(lat, lon, alt);
}

Eigen::Vector3d PayloadSdkInterface::LLA2XYZ(const Eigen::Vector3d &lla) {
  double Ax = 6383487.606;
  double Bx = 5357.31;
  double Ay = 6367449.134;
  double By = 32077.0;
  double dPI = 57.295779512; // 角度转弧度的系数（180/π）
  double lat_a = (neu_pos_init_.x() + lla.x()) / 2.0; // 平均纬度

  double x = ((Ay - By * pow(cos(lat_a / dPI), 2)) * (lla.x() - neu_pos_init_.x())) / dPI;
  double y = -((Ax * cos(lat_a / dPI) - Bx * pow(cos(lat_a / dPI), 3)) * (lla.y() - neu_pos_init_.y())) / dPI;
  double z = lla.z() - neu_pos_init_.z();

  return Eigen::Vector3d(x, y, z);
}