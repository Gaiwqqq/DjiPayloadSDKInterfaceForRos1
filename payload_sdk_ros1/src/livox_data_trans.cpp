#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#define INFO_MSG(str)        do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str)    do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str)  do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str)   do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

// 模拟 livox_pub_ 发布者
ros::Publisher  livox_pub_;
ros::Subscriber livox_sub_;
std::shared_ptr<ros::NodeHandle> nh_;

Eigen::Matrix4d _livox2body_matrix;

template<typename T>
void readParam(std::string param_name, T &param_val, T default_val) {
  if (!nh_->param(param_name, param_val, default_val))
    INFO_MSG_YELLOW("[LIVOX_TRANS] | param: " << param_name << " not found, using default value: " << default_val);
  else
    INFO_MSG_GREEN("[LIVOX_TRANS] | param: " << param_name << " found: " << param_val);
}

// 模拟 livoxCallback 函数
void livoxCallback(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
  if (msg->points.empty()) return;

  ros::Time cur_time = ros::Time::now();
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
  for (auto &point : msg->points) {
    Eigen::Vector4d point_4d(point.x, point.y, point.z, 1.0);
    Eigen::Vector4d point_4d_trans = _livox2body_matrix * point_4d;

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

  ROS_INFO_STREAM_THROTTLE(10.0, "[LIVOX_TRANS]: Livox data recv process spend time : "
          << (ros::Time::now() - cur_time).toSec() * 1e3 << " ms");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_callback_single_executable");
  nh_ = std::make_shared<ros::NodeHandle>("~");

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

  // 初始化发布者
  livox_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("/dji/livox", 2);
  std::string  topic_name;
  readParam<std::string>("dji/livox_sub_topic", topic_name, "/livox/lidar");
  livox_sub_ = nh_->subscribe<livox_ros_driver::CustomMsg>(topic_name, 2, livoxCallback,
                                                           ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

  ros::spin();
  return 0;
}