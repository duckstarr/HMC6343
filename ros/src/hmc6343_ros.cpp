/**
 * @file hmc6343_ros.cpp
 * @author duckstarr
 * @brief ROS Node for HMC6343.
 * 
 */

#include <hmc6343_ros.hpp>
#include <math.h>

namespace sensor
{
  hmc6343_ros::hmc6343_ros(ros::NodeHandle nh, const char * device) : n(nh), hmc6343(device)
  {
    hmc6343_publish_heading = n.advertise<geometry_msgs::QuaternionStamped>("/Quaternion", 1);
    hmc6343_publish_raw_mag = n.advertise<sensor_msgs::MagneticField>("/MagneticField", 1);
  }

  hmc6343_ros::~hmc6343_ros()
  {
    n.shutdown();
    hmc6343_publish_heading.shutdown();
    hmc6343_publish_raw_mag.shutdown();

    ROS_INFO("End of ROS node.");
  }

  void hmc6343_ros::publishHeading()
  {
    hmc6343::readHeading();

    hmc6343_publish_heading.publish(this->toQuaternion((hmc6343::roll / 10.0) * (M_PI / 180.0), 
                                                      (hmc6343::pitch / 10.0) * (M_PI / 180.0), 
                                                      (hmc6343::heading / 10.0) * (M_PI / 180.0)));
  }

  void hmc6343_ros::publishRawMag()
  {
    hmc6343::readMag();

    sensor_msgs::MagneticField mag;
    mag.magnetic_field.x = hmc6343::magX / 10.0;
    mag.magnetic_field.y = hmc6343::magY / 10.0;
    mag.magnetic_field.z = hmc6343::magZ / 10.0;

    hmc6343_publish_raw_mag.publish(mag);
  }

  geometry_msgs::QuaternionStamped hmc6343_ros::toQuaternion(double roll, double pitch, double yaw)
  {
    geometry_msgs::QuaternionStamped quat;

    quat.header.stamp = ros::Time::now();
    
    quat.quaternion.w = std::cos(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) + std::sin(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
    quat.quaternion.x = std::sin(roll/2) * std::cos(pitch/2) * std::cos(yaw/2) - std::cos(roll/2) * std::sin(pitch/2) * std::sin(yaw/2);
    quat.quaternion.y = std::cos(roll/2) * std::sin(pitch/2) * std::cos(yaw/2) + std::sin(roll/2) * std::cos(pitch/2) * std::sin(yaw/2);
    quat.quaternion.z = std::cos(roll/2) * std::cos(pitch/2) * std::sin(yaw/2) - std::sin(roll/2) * std::sin(pitch/2) * std::cos(yaw/2);

    return quat;
  }
}
