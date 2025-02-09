cmake_minimum_required(VERSION 3.8)
project(nav2_dynamic_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(nav2_dynamic_msgs REQUIRED)
find_package(nav2_dynamic_motion_model REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

add_library(${PROJECT_NAME} SHARED
  src/tracked_obstacle_utils.cpp
)
# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    nav2_dynamic_msgs
    nav2_dynamic_motion_model
)

# Export headers
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


add_executable(tracked_obstacle_publisher src/tracked_obstacle_publisher.cpp)
ament_target_dependencies(tracked_obstacle_publisher 
                                  rclcpp 
                                  nav2_dynamic_msgs 
                                  std_msgs 
                                  geometry_msgs 
                                  unique_identifier_msgs 
                                  visualization_msgs)


install(TARGETS tracked_obstacle_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Export package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


ament_package()
