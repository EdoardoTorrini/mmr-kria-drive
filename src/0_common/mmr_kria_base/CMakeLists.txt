cmake_minimum_required(VERSION 3.8)
project(mmr_kria_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CmdMotor.msg"
  "msg/ActuatorStatus.msg"
  "msg/EcuStatus.msg"
  "msg/ResStatus.msg"
  # Add custom msg dependecies to standard ROS2 msgs
  DEPENDENCIES std_msgs
)

install(FILES
  config/configuration.hpp
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}
)

ament_package()