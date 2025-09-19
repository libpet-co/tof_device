cmake_minimum_required(VERSION 3.8)
project(nlink_parser)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_subdirectory(extern/serial)

file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

set(nutils nutils)
add_library(${nutils}
  src/utils/protocol_extracter/nprotocol_base.cpp
  src/utils/protocol_extracter/nprotocol_extracter.cpp
  src/utils/nlink_unpack/nlink_utils.c
  src/utils/nlink_unpack/nlink_linktrack_anchorframe0.c
  src/utils/nlink_unpack/nlink_linktrack_tagframe0.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe0.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe1.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe2.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe3.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe4.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe5.c
  src/utils/nlink_unpack/nlink_linktrack_nodeframe6.c
  src/utils/nlink_unpack/nlink_tofsense_frame0.c
  src/utils/nlink_unpack/nlink_tofsensem_frame0.c
  src/utils/nlink_unpack/nlink_linktrack_aoa_nodeframe0.c
  src/utils/nlink_unpack/nlink_iot_frame0.c
  src/utils/init_serial.cpp
  src/utils/nlink_protocol.cpp
  src/utils/nutils.cpp
)

target_include_directories(${nutils}
  PUBLIC
    src/utils
    extern/serial/include
)

ament_target_dependencies(${nutils}
  rclcpp
  std_msgs
)

target_link_libraries(${nutils}
  serial
)

rosidl_target_interfaces(${nutils}
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

set(linktrack_executable linktrack)
add_executable(${linktrack_executable}
  src/linktrack/init.cpp
  src/linktrack/main.cpp
  src/linktrack/protocols.cpp
)

target_link_libraries(${linktrack_executable}
  ${nutils}
)

ament_target_dependencies(${linktrack_executable}
  rclcpp
  std_msgs
)

rosidl_target_interfaces(${linktrack_executable}
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

set(linktrack_rviz_executable linktrack_rviz_converter)
add_executable(${linktrack_rviz_executable}
  src/linktrack/main_rviz_converter.cpp
)

target_link_libraries(${linktrack_rviz_executable}
  ${nutils}
)

ament_target_dependencies(${linktrack_rviz_executable}
  rclcpp
  geometry_msgs
  nav_msgs
)

rosidl_target_interfaces(${linktrack_rviz_executable}
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

set(tofsense_executable tofsense)
add_executable(${tofsense_executable}
  src/tofsense/init.cpp
  src/tofsense/main.cpp
)

target_link_libraries(${tofsense_executable}
  ${nutils}
)

ament_target_dependencies(${tofsense_executable}
  rclcpp
  std_msgs
)

rosidl_target_interfaces(${tofsense_executable}
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

set(tofsensem_executable tofsensem)
add_executable(${tofsensem_executable}
  src/tofsensem/init.cpp
  src/tofsensem/main.cpp
)

target_link_libraries(${tofsensem_executable}
  ${nutils}
)

ament_target_dependencies(${tofsensem_executable}
  rclcpp
  std_msgs
)

rosidl_target_interfaces(${tofsensem_executable}
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

set(linktrack_aoa_executable linktrack_aoa)
add_executable(${linktrack_aoa_executable}
  src/linktrack_aoa/init.cpp
  src/linktrack_aoa/main.cpp
  src/linktrack/protocols.cpp
)

target_link_libraries(${linktrack_aoa_executable}
  ${nutils}
)

ament_target_dependencies(${linktrack_aoa_executable}
  rclcpp
  std_msgs
)

rosidl_target_interfaces(${linktrack_aoa_executable}
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

set(iot_executable iot)
add_executable(${iot_executable}
  src/iot/init.cpp
  src/iot/main.cpp
)

target_link_libraries(${iot_executable}
  ${nutils}
)

ament_target_dependencies(${iot_executable}
  rclcpp
  std_msgs
)

rosidl_target_interfaces(${iot_executable}
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  ${nutils}
  ${linktrack_executable}
  ${linktrack_rviz_executable}
  ${tofsense_executable}
  ${tofsensem_executable}
  ${linktrack_aoa_executable}
  ${iot_executable}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/tofsensem_fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  grid_map_msgs
  nav_msgs
  rclcpp
  std_msgs
  rosidl_default_runtime
)

ament_package()
