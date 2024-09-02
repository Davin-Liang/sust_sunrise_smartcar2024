cmake_minimum_required(VERSION 3.5)
project(opencv_use)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(ai_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

add_executable(my_opencv src/my_opencv.cpp)
add_executable(racing_track src/racing_track.cpp)
add_executable(line_track src/racing_track_1.cpp)

target_include_directories(my_opencv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(racing_track PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(line_track PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  my_opencv
  "rclcpp"
  "OpenCV"
  # "ai_msgs"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
)

ament_target_dependencies(
  racing_track
  "rclcpp"
  "OpenCV"
  # "ai_msgs"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
)

ament_target_dependencies(
  line_track
  "rclcpp"
  "OpenCV"
  # "ai_msgs"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
)

install(TARGETS my_opencv
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS racing_track
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS line_track
  DESTINATION lib/${PROJECT_NAME})

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
