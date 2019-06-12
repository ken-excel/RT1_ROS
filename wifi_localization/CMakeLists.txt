cmake_minimum_required(VERSION 2.8.3)
project(wifi_nav)

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

add_executable(wifi_node src/wifi_node.cpp)
add_dependencies(wifi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wifi_node ${catkin_LIBRARIES})

add_executable(loc_node src/location_node.cpp)
add_dependencies(loc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(loc_node ${catkin_LIBRARIES})

add_executable(find_loc src/find_location.cpp)
add_dependencies(find_loc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(find_loc ${catkin_LIBRARIES})

add_executable(navigation src/wifi_nav.cpp)
add_dependencies(navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation ${catkin_LIBRARIES})

add_executable(move src/move.cpp)
add_dependencies(move ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move ${catkin_LIBRARIES})

add_executable(straight src/straight.cpp)
add_dependencies(straight ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(straight ${catkin_LIBRARIES})
