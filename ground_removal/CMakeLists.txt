add_library(ground_remove SHARED depth_ground_remover.cpp)
target_include_directories(ground_remove PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(ground_remove
                      # identifiable
                      difference
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS})
