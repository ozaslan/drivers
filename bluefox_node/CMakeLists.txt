cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# ------------------------------------------------------------------------------------ # 
# In order the CMake to work properly, path to driver files (downloaded from Matrix 
# Vision's web site) should be replaced with the path below.
#execute_process(COMMAND tar -xf ${PROJECT_SOURCE_DIR}/bluefox_driver.tgz bluefox_driver)
execute_process(COMMAND tar -C ${PROJECT_SOURCE_DIR} -xf ${PROJECT_SOURCE_DIR}/bluefox_driver.tgz )
#rosbuild_untar_file(./bluefox_driver.tgz ./bluefox_driver ALL)
#set(MVIMPACT_DRIVER_DIR $ENV{ROS_WORKSPACE}/drivers/bluefox_driver)
set(MVIMPACT_DRIVER_DIR bluefox_driver)
set(BLUEFOX_LIBRARY bluefox_camera)

include_directories(${MVIMPACT_DRIVER_DIR})
set(MVIMPACT_LIBRARIES  mvDeviceManager) # mvBlueFOX; mvPropHandling
find_library(MVIMPACT_LIBRARY ${MVIMPACT_LIBRARIES} ${MVIMPACT_DRIVER_DIR}/lib/x86_64/)

rosbuild_add_library(${BLUEFOX_LIBRARY} src/bluefox_camera.cc)
rosbuild_add_executable(${PROJECT_NAME} src/bluefox_node.cc)
target_link_libraries(${PROJECT_NAME} ${BLUEFOX_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${MVIMPACT_LIBRARY})
target_link_libraries(${PROJECT_NAME} yaml-cpp)
