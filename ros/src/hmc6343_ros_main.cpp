/**
 * @file hmc6343_ros_main.cpp
 * @author duckstarr
 * @brief ROS Node for HMC6343.
 * 
 */

#include <hmc6343.hpp>
#include <hmc6343_ros.hpp>

#include <memory>

using namespace sensor;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hmc6343_ros");
  ros::NodeHandle nh("~");

  std::string device;
  bool calibration;

  nh.getParam("device", device);
  nh.getParam("calibration", calibration);

  auto hmc6343_obj = std::unique_ptr<hmc6343_ros>(new hmc6343_ros(nh, device.c_str()));

  while(ros::ok())
  {
    hmc6343_obj->publishRawMag();
    hmc6343_obj->publishHeading();

    ros::Duration(0.2).sleep();
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
