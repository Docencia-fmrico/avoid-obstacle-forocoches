cmake_minimum_required(VERSION 3.8)
project(avoid_obstacle_forocoches)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  kobuki_ros_interfaces
  std_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/avoid_obstacle_forocoches/AvoidObstacleNode.cpp
  src/avoid_obstacle_forocoches/AvoidObstacleAdvancedNode.cpp
  src/avoid_obstacle_forocoches/DebugNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(avoid_obstacle src/avoid_obstacle_main.cpp)
ament_target_dependencies(avoid_obstacle ${dependencies})
target_link_libraries(avoid_obstacle ${PROJECT_NAME})

add_executable(avoid_obstacle_advanced src/avoid_obstacle_advanced_main.cpp)
ament_target_dependencies(avoid_obstacle_advanced ${dependencies})
target_link_libraries(avoid_obstacle_advanced ${PROJECT_NAME})

install(TARGETS
  avoid_obstacle
  avoid_obstacle_advanced
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
