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
  nav_msgs
  tf  # tf 라이브러리 추가
  actionlib
  actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)


##from sanghun
find_package(PCL REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  obj_info.msg
  obj_info2.msg
  obj_array.msg
  average_points.msg
  average_points_array.msg
)

add_service_files(
  FILES
  traffic_srv.srv
  StopTrackingSrv.srv
)

add_action_files(
  FILES
  Planning_Tracking_Act.action
  Person_Collision_Act.action
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
  sensor_msgs
)


catkin_package(
# INCLUDE_DIRS include
  LIBRARIES morai_woowa
  CATKIN_DEPENDS geometry_msgs nav_msgs tf morai_msg roscpp rospy stds_msg message_runtime pcl_conversions pcl_ros actionlib_msgs
#  DEPENDS system_lib
)


include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 ##from sanghun
 ${PCL_INCLUDE_DIRS}
)

add_executable(sensor_main src/sensor/sensor_main.cpp src/sensor/calibration.cpp src/sensor/LiDAR_pre.cpp src/sensor/traffic.cpp src/sensor/calibration2.cpp)
target_link_libraries(sensor_main ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})



add_executable(person_action_node src/sensor/person_action_node.cpp)
target_link_libraries(person_action_node ${catkin_LIBRARIES})

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

add_executable(dwa_class src/planning/dwa_class.cpp)
target_link_libraries(dwa_class
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(gpath_publisher src/planning/gpath_publisher.cpp)
target_link_libraries(gpath_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(path_action_client src/path_action_client.cpp)
add_dependencies(path_action_client ${morai_woowa_EXPORTED_TARGETS})
target_link_libraries(path_action_client ${catkin_LIBRARIES})

##from jimin
add_executable(state_node src/state_node.cpp)
add_dependencies(state_node ${morai_woowa_EXPORTED_TARGETS})
target_link_libraries(state_node ${catkin_LIBRARIES})

add_executable(visualize src/visualize.cpp)
target_link_libraries(visualize ${catkin_LIBRARIES})

##from Gyumin
add_executable(global_path src/planning/global_path.cpp)
target_link_libraries(global_path ${catkin_LIBRARIES})

add_executable(path_tracking src/planning/path_tracking.cpp)
target_link_libraries(path_tracking ${catkin_LIBRARIES})

add_executable(dynamic_planning src/planning/dynamic_planning.cpp)
target_link_libraries(dynamic_planning ${catkin_LIBRARIES})
