# LidarStatus.msg
Header header
bool ConnectState       #通断：0-正常  1-异常
bool SyncTimeState      #时间同步：0-正常  1-异常
bool MotorOneState      #1号电机状态：0-正常  1-异常
bool MotorTwoState      #2号电机状态：0-正常  1-异常
bool EncoderOneState    #1号码盘状态：0-正常  1-异常
bool EncoderTwoState    #2号码盘状态：0-正常  1-异常
bool SysTempState       #系统温度状态：0-正常  1-异常
bool RecTempState       #接收板温度状态：0-正常  1-异常
bool LaserTempState     #激光器温度状态：0-正常  1-异常
bool LaserRayState      #激光器出光状态：0-正常  1-异常
bool RecBiasVolState    #接收偏置电压状态：0-正常  1-异常