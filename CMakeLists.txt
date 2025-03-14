cmake_minimum_required(VERSION 3.16)
project(waveshare_ros2_driver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic -Wshadow
    -Wsign-conversion -Winit-self -Wredundant-decls)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(tl_expected REQUIRED)
find_package(range-v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(transmission_interface REQUIRED)
# Find LibSerial package
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL libserial REQUIRED)

# Set ${PROJECT_NAME}_VERSION
ament_package_xml()

add_library(
  serial_port
  src/serial_port.cpp
)
target_include_directories(serial_port
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${SERIAL_INCLUDE_DIRS}
)
target_link_libraries(serial_port
  PUBLIC
  fmt::fmt
  spdlog::spdlog

  tl_expected::tl_expected
  ${SERIAL_LIBRARIES}
)
set_target_properties(serial_port PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

add_library(
  communication_protocol
  src/communication_protocol.cpp
)
target_include_directories(communication_protocol
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(communication_protocol
  PUBLIC
  serial_port
)
set_target_properties(communication_protocol PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

# add_executable(
#   demo
#   examples/demo.cpp
# )
# target_link_libraries(demo
#   serial_port
#   communication_protocol
# )

add_library(
  ${PROJECT_NAME}
  src/waveshare_ros2_driver.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  hardware_interface
  pluginlib
  transmission_interface
)
target_link_libraries(${PROJECT_NAME}
  communication_protocol
  serial_port
)

pluginlib_export_plugin_description_file(hardware_interface waveshare_ros2_control_plugin_description.xml)

install(
  TARGETS serial_port communication_protocol ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  transmission_interface
)

ament_generate_version_header(${PROJECT_NAME} HEADER_PATH ${PROJECT_NAME}/version.hpp)

ament_package()
