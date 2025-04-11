cmake_minimum_required(VERSION 3.11)
project(depth_clustering)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# needs to be 17+ because i used tuple decomp and maybe other things
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(eigen_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex program_options) # sadge

include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(FLANN REQUIRED)
find_package(Qhull REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ros stuffs
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if (${PCL_FOUND})
  message(STATUS "PCL is found. PCL-related code will be built.")
  # if (${CMAKE_BUILD_TYPE} MATCHES "Debug" AND ${DISTRO} MATCHES "14.04")
  #   message(FATAL_ERROR
  #           "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
  #           "You are building PCL in Debug mode with C++11 enabled.\n"
  #           "This causes segmentation fault due to a bug in PCL.\n"
  #           "Build in Release mode to avoid this bug.\n"
  #           "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
  # endif()
  add_definitions(-DPCL_FOUND)
else()
  message(WARNING "PCL NOT FOUND! PCL-related code WILL NOT be built")
endif()

if(${rclcpp_FOUND})
  message(STATUS "ROS found, building ROS related parts")
  set(ROS_FOUND YES)
else()
  message(WARNING "ROS NOT FOUND. NOT building ROS related parts!")
  set(ROS_FOUND NO)
endif()



add_subdirectory(ground_removal)
add_subdirectory(image_labelers)
add_subdirectory(projections)
add_subdirectory(utils)
add_subdirectory(ros_bridge)


ament_package()
