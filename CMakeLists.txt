cmake_minimum_required(VERSION 3.5)
project(uav_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
SET(OpenCV_DIR /usr/local/include/opencv-3.4.0)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

  sensor_msgs
  geometry_msgs
  mav_msgs
  gazebo_msgs
  image_transport
  cv_bridge
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(control_test src/control_test.cpp)
target_link_libraries(control_test ${catkin_LIBRARIES})

add_executable(video_record src/video_record.cpp)
target_link_libraries(video_record ${catkin_LIBRARIES})

add_executable(img_tracking src/img_tracking.cpp)
add_dependencies(img_tracking ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(img_tracking ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(uav_tracking src/uav_tracking.cpp)
target_link_libraries(uav_tracking ${catkin_LIBRARIES})

add_executable(test_tracking src/test_tracking.cpp)
add_dependencies(test_tracking ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(test_tracking ${catkin_LIBRARIES} ${OpenCV_LIBS})