cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_executable(nao_camera src/nao_camera.cpp)

link_directories(/opt/aldebaran/naoqi-sdk-1.10.52-linux/lib)
target_link_libraries(nao_camera alcommon)
