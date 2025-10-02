# Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

cmake_minimum_required(VERSION 3.16.3)

project(ROSTypeImagePubSubExecutor VERSION 1 LANGUAGES CXX)

find_package(OpenCV REQUIRED)

message(STATUS "Configuring ROSTypeImagePubSubExecutor...")
file(GLOB ROS_TYPE_DATA_PUBLISHER_SOURCES_CXX "src/*.cxx")
file(GLOB ROS_TYPE_DATA_PUBLISHER_SOURCES_CPP "src/*.cpp")


configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml ${CMAKE_CURRENT_BINARY_DIR}/config/config.yaml  COPYONLY)


include_directories(
    include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${ROS_DATA_TYPES_INCLUDE_PATH})
add_executable(ROSTypeImagePubSubExecutor ${ROS_TYPE_DATA_PUBLISHER_SOURCES_CXX} ${ROS_TYPE_DATA_PUBLISHER_SOURCES_CPP} ${in1_files})

target_link_libraries(ROSTypeImagePubSubExecutor ddsc ddscxx lwrcl geometry_msgs sensor_msgs yaml-cpp ${OpenCV_LIBRARIES})
install(TARGETS ROSTypeImagePubSubExecutor
    RUNTIME DESTINATION bin/ROSTypeImagePubSubExecutor/${BIN_INSTALL_DIR})
install(FILES config/config.yaml DESTINATION bin/ROSTypeImagePubSubExecutor/${BIN_INSTALL_DIR}/config PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
