cmake_minimum_required(VERSION 3.8)
project(imx219-83_stereo_camera_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include launch)

# Create executable
add_executable(icm_wrapper_node
                  src/ICM20948_wrapper.cpp
                  src/ICM20948.c)
# Link libraries

# Add dependencies
ament_target_dependencies(icm_wrapper_node
  rclcpp
  sensor_msgs
)

# Install executable
install(TARGETS
  icm_wrapper_node
  DESTINATION lib/${PROJECT_NAME}
)

install (
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

