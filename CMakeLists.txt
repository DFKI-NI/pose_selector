cmake_minimum_required(VERSION 3.0.2)
project(pose_selector)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  object_pose_msgs
  roscpp
  roslib
  std_msgs
  tf2_ros
  vision_msgs
  rospy
)

catkin_python_setup()

add_service_files(
  FILES
  ClassQuery.srv
  ConfigSave.srv
  GetPoses.srv
  PoseDelete.srv
  PoseQuery.srv
  PoseUpdate.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  object_pose_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime object_pose_msgs roscpp roslib std_msgs tf2_ros vision_msgs
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

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS
    dope_converter_node
    ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
