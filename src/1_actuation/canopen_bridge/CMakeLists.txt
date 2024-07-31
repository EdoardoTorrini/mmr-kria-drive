cmake_minimum_required(VERSION 3.8)
project(canopen_bridge)
set (CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(mmr_kria_base REQUIRED)
find_package(mmr_edf REQUIRED)

include_directories(
  include/
)

add_library(motor_maxon src/driver/motor_maxon.cpp)
add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

ament_target_dependencies(motor_maxon
  mmr_kria_base
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  can_msgs
  mmr_kria_base
  mmr_edf
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  motor_maxon
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
