# Copyright (c) 2022，Horizon Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


cmake_minimum_required(VERSION 3.5)
project(racing_control)

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
find_package(std_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})
add_executable(racing_control src/racing_control.cpp)
add_executable(first_patrol src/first_patrol.cpp)
add_executable(second_patrol src/second_patrol.cpp)

ament_target_dependencies(
  first_patrol
  rclcpp
  std_msgs
  geometry_msgs
  ai_msgs
  hbm_img_msgs
)

ament_target_dependencies(
  second_patrol
  rclcpp
  std_msgs
  geometry_msgs
  ai_msgs
  hbm_img_msgs
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  ai_msgs
  hbm_img_msgs
)

target_link_libraries(first_patrol ${CURSES_LIBRARIES})
target_link_libraries(second_patrol ${CURSES_LIBRARIES})

target_include_directories(racing_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(first_patrol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(second_patrol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS racing_control first_patrol second_patrol
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
