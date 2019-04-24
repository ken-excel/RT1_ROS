cmake_minimum_required(VERSION 2.8.3)
project(ros_start)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  message_generation
  tf
  rss
)


add_message_files(
  DIRECTORY msg
  #FILES
  #Rt1Sensor.msg
  #Rt2Sensor.msg
  #Key.msg
  )

add_service_files(
  DIRECTORY srv
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
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs nav_msgs tf rss
 DEPENDS
)

add_executable(rosrt_rt1 src/rosrt_rt1.cpp)
add_dependencies(rosrt_rt1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosrt_rt1 ${catkin_LIBRARIES})

add_executable(rosrt_rt1_tor src/rosrt_rt1_tor.cpp)
add_dependencies(rosrt_rt1_tor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosrt_rt1_tor ${catkin_LIBRARIES})

add_executable(rosrt_rt1_vel src/rosrt_rt1_vel.cpp)
add_dependencies(rosrt_rt1_vel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosrt_rt1_vel ${catkin_LIBRARIES})

add_executable(rosrt_rt1_torvel src/rosrt_rt1_torvel.cpp)
add_dependencies(rosrt_rt1_torvel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosrt_rt1_torvel ${catkin_LIBRARIES})

add_executable(rt1_node src/rt1_node.cpp src/rt1_con.cpp)
add_dependencies(rt1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rt1_node ${catkin_LIBRARIES})

add_executable(wifi_node src/wifi_node.cpp)
add_dependencies(wifi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wifi_node ${catkin_LIBRARIES})

add_executable(loc_node src/location_node.cpp)
add_dependencies(loc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(loc_node ${catkin_LIBRARIES})

add_executable(find_loc src/find_location.cpp)
add_dependencies(find_loc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_loc ${catkin_LIBRARIES})

add_executable(rt1_odom src/rt1_odom.cpp)
add_dependencies(rt1_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rt1_odom ${catkin_LIBRARIES})

add_executable(rt1_nav src/rt1_nav.cpp)
add_dependencies(rt1_nav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rt1_nav ${catkin_LIBRARIES})

add_executable(move src/move.cpp)
add_dependencies(move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move ${catkin_LIBRARIES})
