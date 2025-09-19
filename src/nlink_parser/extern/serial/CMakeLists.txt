cmake_minimum_required(VERSION 3.8)
project(serial)

if(NOT DEFINED SERIAL_EXPORT_SET)
  set(SERIAL_EXPORT_SET serial_export)
endif()

add_library(${PROJECT_NAME}
  src/serial.cc
  include/serial/serial.h
  include/serial/v8stdint.h
)

set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

set(serial_platform_SRCS)

if(APPLE)
  list(APPEND serial_platform_SRCS
    src/impl/unix.cc
    src/impl/list_ports/list_ports_osx.cc
  )
elseif(UNIX)
  list(APPEND serial_platform_SRCS
    src/impl/unix.cc
    src/impl/list_ports/list_ports_linux.cc
  )
else()
  list(APPEND serial_platform_SRCS
    src/impl/win.cc
    src/impl/list_ports/list_ports_win.cc
  )
endif()

if(serial_platform_SRCS)
  target_sources(${PROJECT_NAME} PRIVATE ${serial_platform_SRCS})
endif()

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(APPLE)
  find_library(IOKIT_LIBRARY IOKit)
  find_library(FOUNDATION_LIBRARY Foundation)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
  find_package(Threads REQUIRED)
  target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads rt)
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC setupapi)
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT ${SERIAL_EXPORT_SET}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)
