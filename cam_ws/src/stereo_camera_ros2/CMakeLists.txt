cmake_minimum_required(VERSION 3.5)
project(stereo_camera_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Create executable
add_executable(stereo_camera_node src/stereo_camera_node.cpp)

# Link libraries
target_link_libraries(stereo_camera_node
  ${OpenCV_LIBS}
)

# Add dependencies
ament_target_dependencies(stereo_camera_node
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
)

# Install executable
install(TARGETS
  stereo_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

