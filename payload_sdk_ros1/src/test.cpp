//
// Created by 19260 on 2025/5/8.
//
#include <ros/ros.h>
#include <Eigen/Eigen>

Eigen::Vector3d neu_pos_init_(30.0, 120.0, 100.0);

Eigen::Vector3d XYZ2LLA(const Eigen::Vector3d &xyz) {
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


Eigen::Vector3d LLA2XYZ(const Eigen::Vector3d &lla) {
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  Eigen::Vector3d xyz(0, 0, 0);

  Eigen::Vector3d lla = XYZ2LLA(xyz);
  std::cout << "lla: " << lla.transpose() << std::endl;

  xyz = LLA2XYZ(lla);
  std::cout << "xyz: " << xyz.transpose() << std::endl;
  return 0;
}