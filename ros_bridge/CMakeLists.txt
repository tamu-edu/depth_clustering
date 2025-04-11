add_executable(ground_segment ground_segment.cpp)
target_include_directories(ground_segment PRIVATE ${CMAKE_SOURCE_DIR} ${PCL_INCLUDE_DIRS})
target_link_libraries(ground_segment  ${PCL_LIBRARIES}
                          ${Boost_LIBRARIES}
                          ground_remove
                          projections
                          cloud
                          image_labeler)

ament_target_dependencies(ground_segment
                          rclcpp
                          pcl_conversions
                          pcl_ros
                          sensor_msgs
)
