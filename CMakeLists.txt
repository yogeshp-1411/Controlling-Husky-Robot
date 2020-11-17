cmake_minimum_required(VERSION 3.0.2)
project(deepfield_task)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  roscpp
  std_msgs
  geometry_msgs 
  nav_msgs
  tf
  angles
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES deepfield_task
#  CATKIN_DEPENDS genmsg roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(go_straight src/go_straight.cpp)
target_link_libraries(go_straight ${catkin_LIBRARIES})

add_executable(subscriber_odom src/subscriber_odom.cpp)
target_link_libraries(subscriber_odom ${catkin_LIBRARIES})

add_executable(controlHusky src/controlHusky.cpp)
target_link_libraries(controlHusky ${catkin_LIBRARIES})

