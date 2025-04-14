//
// Created by gwq on 25-4-14.
//
#include "../include/payload_sdk_ros1/payload_sdk_interface.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "payload_sdk_interface");
  ros::NodeHandle nh("~");

  Application application(argc, argv);
  char inputChar;
  T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
  T_DjiReturnCode  returnCode;
  T_DjiTestApplyHighPowerHandler applyHighPowerHandler;

  DjiTest_FcSubscriptionRunSample();

  ros::spin();
  return 0;
}