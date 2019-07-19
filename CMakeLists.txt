# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(realsense_image_raw)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 3.2 REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(data_sub ./src/data_sub.cpp)
target_link_libraries(data_sub
  ${opencv_libraries}
)
ament_target_dependencies(data_sub
	"rclcpp"
       	"sensor_msgs"
        "cv_bridge"
	"ament_index_cpp"
	"class_loader"
)

add_executable(data_pub ./src/data_pub.cpp)
target_link_libraries(data_pub
  ${opencv_libraries}
)
ament_target_dependencies(data_pub
	"rclcpp"
       	"sensor_msgs"
        "cv_bridge"
	"ament_index_cpp"
	"class_loader"
)

install(TARGETS
  data_sub
  data_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
