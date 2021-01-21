cmake_minimum_required(VERSION 2.8.3)
project(uav_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

  sensor_msgs
  geometry_msgs
  mav_msgs
  gazebo_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(control_test src/control_test.cpp)
target_link_libraries(control_test ${catkin_LIBRARIES})

add_executable(video_record src/video_record.cpp)
target_link_libraries(video_record ${catkin_LIBRARIES})
