cmake_minimum_required(VERSION 3.0.2)
project(fusion_slam)
# C++版本
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 使用标准c++,不使用编译器拓展
set(CMAKE_CXX_EXTENSIONS OFF)
# -fexceptions 异常支持 -pthread 线程支持
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread  -fexceptions -O3")
# 生成编译的依赖json文件
set(CMAKE_EXPORT_COMPILE_COMMANDS true)

set(ROS_COMPONENTS 
  tf2
  tf2_ros
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  livox_ros_driver2
  message_generation
  eigen_conversions
)

find_package(catkin REQUIRED COMPONENTS
  ${ROS_COMPONENTS}
)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

catkin_package(
    CATKIN_DEPENDS 
    ${ROS_COMPONENTS}
)
# sophus
include_directories(${PROJECT_SOURCE_DIR}/thirdparty)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CATKIN_DEVEL_PREFIX}/include  # 手动添加devel路径
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# set(SRC_FILE 
#   src/lidar_process.cc
#   src/map_node_build.cc
# )
# add_library( 
#   SRC_LIB  
#   SHARED
#   ${SRC_FILE}
# )
# target_link_libraries(${SRC_LIB}
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   spdlog::spdlog
# )
# target_include_directories(${SRC_LIB}
#   PUBLIC
#   ${catkin_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
# )

add_executable(map_node
  app/map_app.cc
  src/map_node_build.cc
  src/common/common_lib.cc
  src/lidar_process.cc
  src/fastlio_odom/fastlio_odom.cc
  src/fastlio_odom/imu_process.cc
  src/static_imu_init.cc
  include/fastlio_odom/ikd-Tree/ikd_Tree.cpp
)
target_link_libraries(map_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  spdlog::spdlog
)