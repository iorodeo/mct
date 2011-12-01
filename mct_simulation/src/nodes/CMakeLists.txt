# common source files for node driver interface
set(DRIVER_SOURCES 
  sim_camera_driver.cpp
)

rosbuild_add_executable(sim_camera_driver sim_camera_driver_node.cpp ${DRIVER_SOURCES})
