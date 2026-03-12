cmake_minimum_required(VERSION 3.8)
project(st3215_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==============================
# Dependencies
# ==============================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# ==============================
# Create Shared Library
# ==============================

add_library(st3215_hardware_interface SHARED
  src/st3215_system.cpp
  src/SCS.cpp
  src/SCSerial.cpp
  src/SCSCL.cpp
  src/SMSCL.cpp
  src/SMSBL.cpp
  src/SMS_STS.cpp
)

target_include_directories(st3215_hardware_interface PRIVATE
  include
)

ament_target_dependencies(st3215_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
)

# ==============================
# Export Plugin
# ==============================

pluginlib_export_plugin_description_file(
  hardware_interface
  st3215_hardware_interface.xml
)

# ==============================
# Install
# ==============================

install(
  TARGETS st3215_hardware_interface
  DESTINATION lib
)

install(
  FILES st3215_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()