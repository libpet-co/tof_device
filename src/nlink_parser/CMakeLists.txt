cmake_minimum_required(VERSION 3.8)
project(nlink_parser)

# Ensure ccache is disabled for this project so builds do not rely on the
# user-level ccache configuration.
set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "env CCACHE_DISABLE=1")
set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "env CCACHE_DISABLE=1")

# Touch CATKIN_INSTALL_INTO_PREFIX_ROOT if colcon sets it to avoid unused
# variable warnings during configure.
if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
  set(_catkin_install_into_prefix_root "${CATKIN_INSTALL_INTO_PREFIX_ROOT}")
endif()

set(_export_set export_${PROJECT_NAME})
set(SERIAL_EXPORT_SET ${_export_set})

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_environment_hooks(
  ${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/ament_prefix_path.dsv
  ${CMAKE_CURRENT_SOURCE_DIR}/env_hooks/ament_prefix_path.bash
)

add_subdirectory(extern/serial)

file(GLOB msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msg/*.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(rosidl_typesupport_cpp_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/utils>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extern/serial/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}/utils>
)

ament_target_dependencies(${nutils}
  rclcpp
  std_msgs
)

target_link_libraries(${nutils}
  serial
  ${rosidl_typesupport_cpp_target}
)

set_property(TARGET ${nutils} APPEND PROPERTY INTERFACE_LINK_LIBRARIES serial)
set_property(TARGET ${nutils} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${rosidl_typesupport_cpp_target})

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


install(TARGETS
  ${nutils}
  ${linktrack_executable}
  ${linktrack_rviz_executable}
  ${tofsense_executable}
  ${tofsensem_executable}
  ${linktrack_aoa_executable}
  ${iot_executable}
  EXPORT ${_export_set}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY src/utils/
  DESTINATION include/${PROJECT_NAME}/utils
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/tofsensem_fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  env_hooks/ament_prefix_path.dsv
  env_hooks/ament_prefix_path.bash
  DESTINATION share/${PROJECT_NAME}/hook
)

ament_export_targets(${_export_set} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  grid_map_msgs
  nav_msgs
  rclcpp
  std_msgs
  rosidl_default_runtime
)

ament_package()

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/env_hooks)

configure_file(
  env_hooks/package.dsv.in
  ${CMAKE_CURRENT_BINARY_DIR}/env_hooks/package.dsv
  @ONLY
)

install(CODE "
  file(COPY \"${CMAKE_CURRENT_BINARY_DIR}/env_hooks/package.dsv\"
    DESTINATION \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\")
")

set(_nlink_parser_package_dsv_append "source;share/${PROJECT_NAME}/local_setup.bash\nsource;share/${PROJECT_NAME}/local_setup.sh\nsource;share/${PROJECT_NAME}/local_setup.dsv\nsource;share/${PROJECT_NAME}/local_setup.zsh\n")

install(CODE "
  file(APPEND \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/package.dsv\" \"${_nlink_parser_package_dsv_append}\")
")

install(CODE [[
  set(_nlink_parser_local_setup "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/local_setup.bash")
  if(EXISTS "${_nlink_parser_local_setup}")
    file(READ "${_nlink_parser_local_setup}" _nlink_parser_local_setup_content)
    if(NOT _nlink_parser_local_setup_content MATCHES "AMENT_PREFIX_PATH")
      set(_needle "_colcon_prefix_bash_prepend_unique_value COLCON_PREFIX_PATH \"\$_colcon_prefix_bash_COLCON_CURRENT_PREFIX\"")
      set(_replacement "${_needle}\n_colcon_prefix_bash_prepend_unique_value AMENT_PREFIX_PATH \"\$_colcon_prefix_bash_COLCON_CURRENT_PREFIX\"")
      string(REPLACE "${_needle}" "${_replacement}" _nlink_parser_local_setup_content "${_nlink_parser_local_setup_content}")
    endif()
    file(WRITE "${_nlink_parser_local_setup}" "${_nlink_parser_local_setup_content}")
  endif()
]])
