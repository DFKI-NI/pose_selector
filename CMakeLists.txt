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
)

add_service_files(
  FILES
  PoseQuery.srv
  ClassQuery.srv
  PoseUpdate.srv
  PoseDelete.srv
  ConfigSave.srv
)

add_message_files(
  FILES
  LabeledPose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime roslib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/pose_selector_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

add_executable(update_test src/pose_update_tester.cpp)
add_dependencies(update_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(update_test
  ${catkin_LIBRARIES}
)
