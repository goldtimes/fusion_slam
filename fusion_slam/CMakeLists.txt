cmake_minimum_required(VERSION 3.0.2)
project(fusion_slam)
# C++版本
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 使用标准c++,不使用编译器拓展
set(CMAKE_CXX_EXTENSIONS OFF)
# -fexceptions 异常支持 -pthread 线程支持
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -fexceptions")
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

# set(SRC_FILE)

add_executable(map_build_node app/map_build_node.cc)
target_link_libraries(map_build_node
  ${catkin_LIBRARIES}
  spdlog::spdlog
)