cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(mmr_kria_base REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)



#_________________________________________________________
#                    WAYPOINTS SPLINE
#_________________________________________________________

add_library(waypoint_spline src/waypoints_spline.cpp)

ament_target_dependencies(waypoint_spline
  rclcpp
  visualization_msgs
  mmr_kria_base
)


#_________________________________________________________
#                     PURE PURESUIT
#_________________________________________________________


add_library(${PROJECT_NAME} src/pure_pursuit.cpp)

target_link_libraries(${PROJECT_NAME}
  waypoint_spline
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  visualization_msgs
  ackermann_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  mmr_kria_base
)

#_________________________________________________________
#                    PURE PURSUIT NODE
#_________________________________________________________


add_executable(${PROJECT_NAME}_node src/pure_pursuit_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  visualization_msgs
  ackermann_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  mmr_kria_base
)

#_________________________________________________________
#                 INCLUDE DIRECTORYS AND LIBS
#_________________________________________________________


include_directories(
                include
)

# Install target
install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
        launch/
        DESTINATION share/${PROJECT_NAME}/launch
    )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
