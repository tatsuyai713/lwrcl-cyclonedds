cmake_minimum_required(VERSION 3.16.3)
project(lwrcl)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64" )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
endif()

if(${CMAKE_SYSTEM_NAME} STREQUAL "QNX")
    message("|||||||||QNX||||||||")
    include_directories(/opt/qnx/cyclonedds/include)
    include_directories(/opt/qnx/cyclonedds/include/ddscxx)
    include_directories(/opt/qnx/cyclonedds/include)
    # std_msgsのパスを指定
    include_directories(/opt/qnx/cyclonedds-libs/include)
    link_directories(/opt/qnx/cyclonedds-libs/lib)
    link_directories(/opt/qnx/cyclonedds/lib)
else()
    include_directories(/opt/cyclonedds/include)
    include_directories(/opt/cyclonedds/include/ddscxx)
    link_directories(/opt/cyclonedds/lib)
    # std_msgsのパスを指定
    include_directories(/opt/cyclonedds-libs/include)
    link_directories(/opt/cyclonedds-libs/lib)
    link_directories(/opt/qnx/cyclonedds/lib)
endif()

add_subdirectory(src/yaml-cpp)