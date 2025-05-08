#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <Eigen/Eigen>
#include <mavros_msgs/PositionTarget.h>
#include <flyctrl/flyctrl_send.h>

ros::Publisher setpoint_pub;

struct Order {
double vx;
double vy;
double vz;
double yaw_rate;
};

Order fly_order;

void controlCallback(const flyctrl::flyctrl_send::ConstPtr& msg)
{
   
    fly_order.vx =  msg->u_sp ;
    fly_order.vy =  -msg->v_sp ;
    fly_order.vz =  msg->w_sp ;
    fly_order.yaw_rate =  -msg->r_sp /180 *M_PI;
}

void mavros_pub(const ros::TimerEvent& event)
{
    mavros_msgs::PositionTarget setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.frame_id = "map";
    // 设置坐标系
    setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    // 设置 type_mask，仅关注 VX、VY、VZ 和 YAW_RATE
    setpoint.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW;

    setpoint.velocity.x = fly_order.vx;
    setpoint.velocity.y = fly_order.vy;
    setpoint.velocity.z = fly_order.vz;
    setpoint.yaw_rate   = fly_order.yaw_rate;


    std::cout<<"vx: "<<setpoint.velocity.x<<" "<<"vy: "<<setpoint.velocity.y<<" "<<"vz: "<<setpoint.velocity.z<<" "<<"yaw_rate: "<<setpoint.yaw_rate<<std::endl;

    setpoint_pub.publish(setpoint);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "flyctrl_pub");
    ros::NodeHandle nh("~");
 
    //接收60发布的控制指令
    ros::Subscriber control_sub = nh.subscribe("/flyctrl_send", 10, controlCallback);
    setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ros::Timer control_timer_ = nh.createTimer(ros::Duration(0.1), mavros_pub);

  ros::spin();
  return 0;
}
