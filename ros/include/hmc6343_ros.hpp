/**
 * @file hmc6343_ros.hpp
 * @author duckstarr
 * @brief ROS Node for HMC6343.
 * 
 */

#ifndef HMC6343_ROS_HPP
#define HMC6343_ROS_HPP

#include <hmc6343.hpp>

#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/MagneticField.h>

namespace sensor
{
  class hmc6343_ros : public hmc6343
  {
    public:
      /**
       * @brief Construct a new hmc6343 ros object
       * 
       * @param nh 
       * @param device 
       */
      hmc6343_ros(ros::NodeHandle nh, const char * device);

      /**
       * @brief Virtual destructor
       * 
       */
      virtual ~hmc6343_ros();

      /**
       * @brief Disable copy constructor
       * 
       * @param ihmc6343 
       */
      hmc6343_ros(hmc6343_ros& ihmc6343) = delete;

      /**
       * @brief Disable copy assignment
       * 
       * @param ihmc6343 
       */
      void operator=(hmc6343_ros& ihmc6343) = delete;

      /**
       * @brief Publish the heading information
       * 
       */
      void publishHeading();

      /**
       * @brief Publish magnetic field information
       * 
       */
      void publishRawMag();

    private:
      /**
       * @brief Convert Eruler's representation to Quaternion
       * 
       * @param roll 
       * @param pitch 
       * @param yaw 
       * @return geometry_msgs::QuaternionStamped 
       */
      geometry_msgs::QuaternionStamped toQuaternion(double roll, double pitch, double yaw);

    private:
      ros::NodeHandle n;                      // Nodehandler
      ros::Publisher hmc6343_publish_heading; // Publisher object for heading information
      ros::Publisher hmc6343_publish_raw_mag; // Publisher object for magnetic field information
  };
}

#endif /* HMC6343_ROS_HPP */
