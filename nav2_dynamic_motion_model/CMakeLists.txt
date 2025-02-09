cmake_minimum_required(VERSION 3.8)
project(nav2_dynamic_motion_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_dynamic_msgs REQUIRED)



include_directories(include)

add_library(nav2_dynamic_motion_model 
    src/constant_velocity_model.cpp
)
ament_target_dependencies(nav2_dynamic_motion_model
    rclcpp
    nav2_dynamic_msgs
    geometry_msgs
)
# Export header files
# target_include_directories(nav2_dynamic_motion_model PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
# Install libraries
install(
    TARGETS nav2_dynamic_motion_model
    EXPORT export_nav2_dynamic_motion_model
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)


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

# Export library for other packages
ament_export_include_directories(include)
ament_export_libraries(nav2_dynamic_motion_model)

ament_package()
