cmake_minimum_required(VERSION 3.16.3)
project(lwrcl)


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


add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME} ddsc ddscxx yaml-cpp)

# Install targets
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Install headers
install(FILES 
include/lwrcl.hpp 
include/fast_dds_header.hpp
include/publisher.hpp 
include/subscription.hpp 
include/clock_time_duration.hpp
include/timer.hpp 
include/channel.hpp
include/qos.hpp
DESTINATION include/)

install(FILES
include/rclcpp/rclcpp.hpp
include/rclcpp/qos.hpp
DESTINATION include/rclcpp)

