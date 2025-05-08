std_msgs/Header header
uint8 state     # 0x00死亡，其他自定义，想不好就0x01正常
uint8 rosandimu     # 0x00时间未同步，时间已同步
uint8 rosandlidar     # 0x00时间未同步，时间已同步
uint8 imuandlidar     # 0x00时间未同步，时间已同步
uint8 imu_fre         # 惯导帧频
uint8 imu_state     # 惯导状态字
uint32 imu_cnt     # 惯导帧计数器，为方便下传这里只取一个字节
uint8 imu_fault     #惯导判故状态 0-无故障，1-惯导故障
uint8 origin_pos_flag #0-未获取原点，1-已获取原点
float32 origin_longitude #原点经度
float32 origin_latitude #原点纬度
float32 origin_altitude #原点高度
uint8 lidar_fre         # 雷达帧频
uint32 lidar_cnt         # 雷达帧计数器，为方便下传这里只取一个字节
uint8 lidar_fault         # 雷达故障状态
uint8 params_fault         # 参数加载状态
