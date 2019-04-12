cmake_minimum_required(VERSION 2.8.3)
project(rplidar_ros)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  )

generate_messages(
    DEPENDENCIES
    std_msgs
    )

include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
 CATKIN_DEPENDS roscpp message_runtime geometry_msgs std_msgs 
 EXPORTED_TARGETS rplidar_ros_generate_messages_cpp
)


add_executable(rplidarNode src/node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidarNode ${catkin_LIBRARIES})

add_executable(rplidarNodeClient src/client.cpp)
target_link_libraries(rplidarNodeClient ${catkin_LIBRARIES})

#add_executable(obsdetection src/obstacle_detect.cpp)
#target_link_libraries(obsdetection ${catkin_LIBRARIES})

#add_executable(obsdetect_angled src/obsta_angled.cpp)
#target_link_libraries(obsdetect_angled ${catkin_LIBRARIES})

add_executable(angled_rplidar src/angled_rplidar.cpp src/angled_rplidar_node.cpp)
add_dependencies(angled_rplidar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(angled_rplidar ${catkin_LIBRARIES})

install(TARGETS rplidarNode rplidarNodeClient
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
