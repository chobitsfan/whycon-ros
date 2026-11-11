cmake_minimum_required(VERSION 3.16)
project(whycon CXX)

# Plain CMake build against an installed ROS 2 (source /opt/ros/<distro>/setup.bash first).

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g")

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(whycon
    src/whycon_ros/whycon_ros_node.cpp
    src/whycon/CWhycon.cpp
    src/whycon/CCircleDetect.cpp
    src/whycon/CRawImage.cpp
    src/whycon/CTimer.cpp
    src/whycon/CTransformation.cpp
    src/whycon/CNecklace.cpp
    )

target_include_directories(whycon PRIVATE include ${OpenCV_INCLUDE_DIRS})

target_link_libraries(whycon
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    tf2_ros::tf2_ros
    ${OpenCV_LIBS}
    )

install(TARGETS whycon RUNTIME DESTINATION bin)
