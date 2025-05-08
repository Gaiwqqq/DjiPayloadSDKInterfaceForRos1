std_msgs/Header header 
uint8 state	#0x00 0x01 使用了，后续自定义
uint32 ctrl_imu_id #imu帧序号
uint8  ctrl_imu_fre #imu帧频
uint8  ctrl_imu_fault #imu故障
uint8  ctrl_imu_state #imu状态字
uint32 ctrl_lpp_id #规划id
uint8  ctrl_lpp_fre #规划帧频
uint8  ctrl_lpp_fault #规划故障
uint32 ctrl_radar_id #防撞雷达id
uint8  ctrl_radar_fre #防撞雷达帧频
uint8  ctrl_radar_fault #防撞雷达故障
uint8  ctrl_radar_state #防撞雷达状态
uint32 ctrl_receivedata_id #接收飞控参数id
uint8  ctrl_receivedata_fre #接收飞控参数帧频
uint8  ctrl_receivedata_fault #接收飞控参数故障状态