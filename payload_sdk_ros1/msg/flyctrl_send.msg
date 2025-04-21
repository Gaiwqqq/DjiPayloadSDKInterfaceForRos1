std_msgs/Header header
uint8 switch_stickground #贴地开关
uint8 switch_sticksea #掠海开关
uint8 switch_lpp #避障开关
uint8 switch_bag #录包开关
uint8 fly_state #任务飞行状态  0=航线飞行、1=航线绕障、2=航线防撞、3=航线贴地、4=航线掠海、5=航线贴地绕障
float32 u_sp #纵向速度指令
float32 v_sp #侧向速度指令
float32 w_sp #垂速指令
float32 r_sp #偏航角速度指令
float32 yaw_sp #航向角指令
float32 h_sp  #高度设定
float32 u_pv #有待删除：纵向速度反馈
float32 v_pv #有待删除：侧向速度反馈
float32 w_pv #有待删除：垂速反馈
float32 r_pv #有待删除：偏航角速度反馈
float32 yaw_pv #有待删除：航向角
float32 h_pv #有待删除：海拔高度
float32 longitude  #飞行经度
float32 latitude   #飞行纬度
float32 longitude_tar  #目标经度
float32 latitude_tar   #目标纬度
uint8 routeId   #当前航线号，用于飞控接手时的切入目标
uint8 pointId    #当前航点号，用于飞控接手时的切入目标
uint8 ObsStatus1 #贴地避障监测状态1
uint8 ObsStatus2 #贴地避障监测状态2
uint8 ObsStatus3 #贴地避障监测状态3
uint8 ObsStatus4 #贴地避障监测状态4
uint8 RecordStatus #记录状态
uint8 lpp_algorithm #规划算法1-跟踪全局，2-归边航线， 3-虚拟点，4-A星航线
float32 dist_radar_orig #防撞雷达原始距离
float32 dist_radar_filter #防撞雷达滤波距离
float32 alti_relativeground #对地控制高度
float32 alti_radio #无线电高度
uint8 fre_flystatus #飞行状态帧频 
uint8 fre_imu #惯导帧频
uint8 fre_radar #防撞雷达帧频
uint8 fre_lidar #避障雷达帧频
uint8 fre_mapheart #建图心跳帧频
uint8 fre_lppheart #规划心跳帧频
uint8 fre_mapping #建图输出帧频
uint8 fre_lpp #规划输出帧频


