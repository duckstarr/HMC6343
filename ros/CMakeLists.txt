cmake_minimum_required(VERSION 3.2)
project(hmc6343_ros)

# Find packages.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

# Catkin specific configuration.
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(include 
  ../include/
  ../utilities/
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/hmc6343_ros.cpp
  src/hmc6343_ros_main.cpp

  ../src/hmc6343.cpp
  ../src/i2c.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Install 
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
