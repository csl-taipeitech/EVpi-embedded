cmake_minimum_required(VERSION 3.5)
project(evpi_bringup)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)             

################################################################################
# Build
################################################################################
include_directories(
  include
)

add_executable(can_manager src/can_manager.cpp)
ament_target_dependencies(can_manager rclcpp can_msgs) 

################################################################################
# Install
################################################################################
install(DIRECTORY 
  launch config include
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  can_manager
  DESTINATION lib/${PROJECT_NAME})

################################################################################
# Macro for ament package
################################################################################
ament_package()
