cmake_minimum_required(VERSION 2.4.8)
find_package( OpenCV REQUIRED )
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
project( QRCode )


# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/mavlink)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(GUIDANCE_PACK_SRCS 
  src/pid.cpp
  src/guidance_pack.cpp
)
#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(guidance_pack ${GUIDANCE_PACK_SRCS})
rosbuild_add_executable(optical_flow src/optical_flow.cpp)
rosbuild_add_executable(qr_code src/qr_code.cpp)
rosbuild_add_executable(server src/server.cpp)

target_link_libraries (qr_code  ${ZBAR_LIBRARIES}    ${OpenCV_LIBS} zbar)





#target_link_libraries(example ${PROJECT_NAME})
