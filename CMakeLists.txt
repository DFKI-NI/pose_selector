cmake_minimum_required(VERSION 3.0.2)
project(pose_selector)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  message_generation
  tf2_ros
  object_pose_msgs
  vision_msgs
)

add_service_files(
  FILES
  PoseQuery.srv
  ClassQuery.srv
  PoseUpdate.srv
  PoseDelete.srv
  ConfigSave.srv
  GetPoses.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  object_pose_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy geometry_msgs std_msgs message_runtime roslib object_pose_msgs vision_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/pose_selector_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(dope_converter_node src/dope_to_pose_node.cpp)
target_link_libraries(dope_converter_node
  ${catkin_LIBRARIES}
)
add_dependencies(dope_converter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

