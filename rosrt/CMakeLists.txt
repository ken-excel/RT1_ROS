cmake_minimum_required(VERSION 2.8.3)
project(ros_start)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)


add_message_files(
  DIRECTORY msg
  #FILES
  #Rt1Sensor.msg
  #Rt2Sensor.msg
  #Key.msg
  )

generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

catkin_package(
 INCLUDE_DIRS 
 LIBRARIES 
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
 DEPENDS
)

add_executable(rosrt_rt1 src/rosrt_rt1.cpp)
target_link_libraries(rosrt_rt1 ${catkin_LIBRARIES})
add_dependencies(rosrt_rt1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rt1_node src/rt1_node.cpp src/rt1_con.cpp)
target_link_libraries(rt1_node ${catkin_LIBRARIES})
add_dependencies(rt1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(rosrt_rt2 src/rosrt_rt2.cpp)
target_link_libraries(rosrt_rt2 ${catkin_LIBRARIES})
add_dependencies(rosrt_rt2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


