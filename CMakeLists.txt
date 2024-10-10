cmake_minimum_required(VERSION 3.0.2)
project(morai_woowa)

find_package(catkin REQUIRED COMPONENTS
  morai_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
  sensor_msgs
  geometry_msgs
  tf  # tf 라이브러리 추가
)

find_package(OpenCV REQUIRED)


##from sanghun
find_package(PCL REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  obj_info.msg
)

# add_service_files(
#   FILES
# )


# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES morai_woowa
  CATKIN_DEPENDS morai_msg roscpp rospy stds_msg message_runtime pcl_conversions pcl_ros
#  DEPENDS system_lib
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 ##from sanghun
 ${PCL_INCLUDE_DIRS}
)

add_executable(calibration src/sensor/calibration.cpp src/sensor/LiDAR_pre.cpp src/sensor/traffic.cpp)
target_link_libraries(calibration ${catkin_LIBRARIES} ${OpenCV_LIBS})


##from sanghun
add_executable(lidar_db src/planning/lidar_db.cpp)
target_link_libraries(lidar_db
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(obstacle_tracking src/planning/obstacle_tracking.cpp)
target_link_libraries(obstacle_tracking
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(dwa src/planning/dwa.cpp)
target_link_libraries(dwa
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

##from jimin
add_executable(state_node src/state_node.cpp)
target_link_libraries(state_node ${catkin_LIBRARIES})

add_executable(visualize src/visualize.cpp)
target_link_libraries(visualize ${catkin_LIBRARIES})

