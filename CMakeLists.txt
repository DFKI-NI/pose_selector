cmake_minimum_required(VERSION 3.8)
project(pose_selector)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(object_pose_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

set(srv_files
  "srv/ClassQuery.srv"
  "srv/ConfigSave.srv"
  "srv/GetPoses.srv"
  "srv/PoseDelete.srv"
  "srv/PoseQuery.srv"
  "srv/PoseUpdate.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES object_pose_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(${PROJECT_NAME}_node src/pose_selector_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  ament_index_cpp
  geometry_msgs
  object_pose_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  vision_msgs
)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}__rosidl_generator_cpp)

add_executable(dope_converter_node src/dope_to_pose_node.cpp)
ament_target_dependencies(dope_converter_node
  rclcpp
  ament_index_cpp
  geometry_msgs
  object_pose_msgs
  vision_msgs
)
add_dependencies(dope_converter_node ${PROJECT_NAME}__rosidl_generator_cpp)

add_executable(pose_selector_visualizer_node src/pose_selector_visualizer_node.cpp)
ament_target_dependencies(pose_selector_visualizer_node
  rclcpp
  ament_index_cpp
  visualization_msgs
)
add_dependencies(pose_selector_visualizer_node ${PROJECT_NAME}__rosidl_generator_cpp)

install(TARGETS
  ${PROJECT_NAME}_node
  dope_converter_node
  pose_selector_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  example_meshes
  srv
  DESTINATION share/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}" yaml-cpp)
target_link_libraries(dope_converter_node yaml-cpp)
target_link_libraries(pose_selector_visualizer_node "${cpp_typesupport_target}" yaml-cpp)

ament_package()
