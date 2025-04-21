std_msgs/Header header
float32 airSpeed  #空速
float32 groundSpeed #地速
float32 longitude  #飞行经度
float32 latitude   #飞行纬度
float32 altitude   #飞行海拔高度
uint8 routeId   #当前航线号
uint8 pointId    #当前航点号
float32 radioHeight  #无线电高度
uint8 radioValid     #无线电高度有效性  0=无效，1=有效。
uint8 switch_stickground #贴地开关
uint8 switch_sticksea #掠海开关
uint8 switch_lpp #避障开关
uint8 switch_bag #录包开关
uint8 fly_state #任务飞行状态  0=航线飞行、1=航线绕障、2=航线防撞、3=航线贴地、4=航线掠海、5=航线贴地绕障
float32 u_sp #纵向速度指令反馈
float32 v_sp #侧向速度指令反馈
float32 w_sp #垂速指令反馈
float32 r_sp #偏航角速度指令反馈
float32 yaw_sp #航向角指令反馈
float32 h_sp  #高度设定
float32 u_pv #纵向速度反馈
float32 v_pv #侧向速度反馈
float32 w_pv #垂速反馈
float32 r_pv #偏航角速度反馈
float32 yaw_pv #航向角
float32 h_pv #海拔高度
uint8 flag_groundair #空地标志：0=地面，1=空中
uint16 eng_spd #发动机转速
uint8 flyline_status #航线飞行状态（具体定义飞控给出）
uint8 device_power  #任务设备上电标志
