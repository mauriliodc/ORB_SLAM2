#__/\\\\\\\\\\\__/\\\\\\_____________________/\\\\\\\\\__________________________________________________________        
# _\/////\\\///__\////\\\__________________/\\\////////___________________________________________________________       
#  _____\/\\\________\/\\\________________/\\\/____________________________________________________________________      
#   _____\/\\\________\/\\\_______________/\\\______________/\\\\\\\\\_____/\\\\\\\\\\\__/\\\\\\\\\\\_____/\\\\\____     
#    _____\/\\\________\/\\\______________\/\\\_____________\////////\\\___\///////\\\/__\///////\\\/____/\\\///\\\__    
#     _____\/\\\________\/\\\______________\//\\\______________/\\\\\\\\\\_______/\\\/_________/\\\/_____/\\\__\//\\\_   
#      _____\/\\\________\/\\\_______________\///\\\___________/\\\/////\\\_____/\\\/_________/\\\/______\//\\\__/\\\__  
#       __/\\\\\\\\\\\__/\\\\\\\\\______________\////\\\\\\\\\_\//\\\\\\\\/\\__/\\\\\\\\\\\__/\\\\\\\\\\\__\///\\\\\/___ 
#        _\///////////__\/////////__________________\/////////___\////////\//__\///////////__\///////////_____\/////_____


cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()


find_package(catkin REQUIRED COMPONENTS
  thin_state_publisher
  tf
  tf_conversions
)

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 2.4.3 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
)

# Node for monocular camera
#rosbuild_add_executable(Mono
#src/ros_mono.cc
#)
#
#target_link_libraries(Mono
#${LIBS}
#)
# Node for stereo camera
#rosbuild_add_executable(Stereo
#src/ros_stereo.cc
#)
#target_link_libraries(Stereo
#${LIBS}
#)
# Node for RGB-D camera
#rosbuild_add_executable(RGBD
#src/ros_rgbd.cc
#)
#target_link_libraries(RGBD
#${LIBS}
#)
# ██████╗ ██╗   ██╗ █████╗ ██████╗ ██████╗  █████╗      ██████╗ ██╗   ██╗ █████╗     
#██╔════╝ ██║   ██║██╔══██╗██╔══██╗██╔══██╗██╔══██╗    ██╔═══██╗██║   ██║██╔══██╗    
#██║  ███╗██║   ██║███████║██████╔╝██║  ██║███████║    ██║   ██║██║   ██║███████║    
#██║   ██║██║   ██║██╔══██║██╔══██╗██║  ██║██╔══██║    ██║▄▄ ██║██║   ██║██╔══██║    
#╚██████╔╝╚██████╔╝██║  ██║██║  ██║██████╔╝██║  ██║    ╚██████╔╝╚██████╔╝██║  ██║    
# ╚═════╝  ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚═════╝ ╚═╝  ╚═╝     ╚══▀▀═╝  ╚═════╝ ╚═╝  ╚═╝    
                                                                                    

# Node XTION+JOINT STATE
rosbuild_add_executable(XTION
src/ros_rgbd.cc
)

target_link_libraries(XTION
${catkin_LIBRARIES}
${LIBS}
)
add_dependencies(XTION ${catkin_EXPORTED_TARGETS})



#SINGLE 3D Sensor
rosbuild_add_executable(new_world_so3_node src/new_world_so3_node.cpp)
target_link_libraries(new_world_so3_node
${LIBS} ${catkin_LIBRARIES}
)
add_dependencies(new_world_so3_node ${catkin_EXPORTED_TARGETS})
