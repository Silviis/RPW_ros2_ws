cmake_minimum_required(VERSION 3.8)
project(imx219-83_stereo_camera_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wunused)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  launch
  ${OpenCV_INCLUDE_DIRS}
)

# Create executable
add_executable(stereo_camera_node src/stereo_camera_node.cpp)

# Create executable
add_executable(icm20948_node src/icm20948_node.cpp)

# Link libraries
target_link_libraries(stereo_camera_node
  ${OpenCV_LIBS}
)

# Add dependencies
ament_target_dependencies(icm20948_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(stereo_camera_node
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
)
# Install executable
install(TARGETS
  icm20948_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  stereo_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install (
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install (
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

