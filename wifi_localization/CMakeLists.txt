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

add_executable(navigation_re src/wifi_nav_recursive.cpp)
add_dependencies(navigation_re ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation_re ${catkin_LIBRARIES})

add_executable(measure src/measure.cpp)
add_dependencies(measure ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(measure ${catkin_LIBRARIES})

add_executable(wifi_database src/wifi_database.cpp)
add_dependencies(wifi_database ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wifi_database sqlite3 ${catkin_LIBRARIES})

add_executable(wifi_nav_goal src/wifi_nav_goal.cpp)
add_dependencies(wifi_nav_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wifi_nav_goal sqlite3 ${catkin_LIBRARIES})

add_executable(wifi_nav_goal_sim src/wifi_nav_goal_sim.cpp)
add_dependencies(wifi_nav_goal_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wifi_nav_goal_sim sqlite3 ${catkin_LIBRARIES})

add_executable(traj src/trajectory.cpp)
add_dependencies(traj ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj sqlite3 ${catkin_LIBRARIES})
