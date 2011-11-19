# common source files for node driver interface
set(DRIVER_SOURCES 
  camera_info_driver.cpp
)

rosbuild_add_executable(simulated_camera_info_node simulated_camera_info_node.cpp ${DRIVER_SOURCES})
# target_link_libraries(camera1394_node ${DC1394})
