std_msgs/Header header 
uint8 state	#0x00 0x01 使用了，后续自定义
uint8 lpp_method #归边、A*等
uint32 lpp_imu_id #imu帧序号
uint8  lpp_imu_fre #imu帧频
uint8  lpp_imu_fault #imu故障
uint8  lpp_imu_state #imu状态字
uint32 lpp_3d_id #imu帧序号
uint8  lpp_3d_fre #imu帧频
uint8  lpp_3d_fault #imu故障
uint8 lpp_params_fault         # 参数加载状态
uint8 CurRoutePointIndex #在当前航路中的航点号
uint8 CurRouteIndex #当前航路号