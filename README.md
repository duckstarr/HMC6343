**HMC6343**
====
The Honeywell HMC6343 is a cutting-edge compass module that offers unmatched accuracy and performance. With its combination of a three-axis magneto-resistive sensor, three-axis MEMS accelerometer, and powerful processing electronics, this module delivers reliable and precise heading computation. The compact 9.0mm x 9.0mm x 1.9mm LCC package makes it easy to integrate into a wide range of applications, including binoculars, cameras, night vision optics, laser ranger finders, antenna positioning, and industrial compassing.

This module also features Honeywell's Anisotropic Magnetoresistive (AMR) technology, which provides a level of precision sensitivity and linearity that sets it apart from other magnetic sensor technologies. The solid-state construction and low cross-axis sensitivity allow for accurate measurement of both the direction and magnitude of Earth's magnetic field. With these features, the HMC6343 is a cost-effective and reliable solution for all your compass needs.

**About**
====
This project contains C++ examples that demonstrate how to interface with the HMC6343 module through an I2C protocol with a Linux-based OS using the [Standard C POSIX library](https://en.wikipedia.org/wiki/C_POSIX_library). This project acquires accelerometer, magnetometer, heading, and tilt data from the HMC6343 and leverages [ROS](https://www.ros.org/) for potential use in navigation applications.


**Docker Usage**
====
1. Pulling the Docker container.

  ```
  docker pull duckstarr/hmc6343:arm32v7
  ```

2. Deploying (executing) the Docker container.

  ```
  docker run -it --rm --privileged --name hmc6343 \
    duckstarr/hmc6343:arm32v7 \
    roslaunch hmc6343_ros hmc6343.launch
  ```

**HMC6343 API**
====
**Published Topics**

~< name >/Quaternion ([geometry_msgs/QuaternionStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/QuaternionStamped.html))
- The orientation of the HMC6343 Compass in quaternion coordinate.

~< name >/MagneticField ([sensor_msgs/MagneticField](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html))
- The magnetic field strength read by HMC6353.

**Parameters**

~< name >/device (string, default: /dev/i2c-1)
- The aboslute path to HMC6343 device.
