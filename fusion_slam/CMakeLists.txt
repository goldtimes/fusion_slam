cmake_minimum_required(VERSION 3.16)
project(fusion_slam)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)
# 添加编译选项
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -rdynamic")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -std=c++0x -std=c++17 -fexceptions")

find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)
# 查找yaml-cpp库
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS stacktrace_backtrace filesystem)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
# if(YAML_CPP_FOUND)
#     message(STATUS "Found yaml-cpp version: ${YAML_CPP_VERSION}")
#     message(STATUS "Using yaml-cpp include dirs: ${YAML_CPP_INCLUDE_DIRS}")
#     include_directories(${YAML_CPP_INCLUDE_DIRS})
# else()
#     message(FATAL_ERROR "Could not find yaml-cpp")
# endif()
add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=3)
message("core for MP: 3")
# 增加omp库的使用
find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_generation
  tf
  livox_ros_driver2
)

add_subdirectory(include/3rd/livox_ros_driver)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fusion_slam
 CATKIN_DEPENDS
#  DEPENDS system_lib
  roscpp
  sensor_msgs
  message_runtime
  tf
  livox_ros_driver2
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# sophus 找到自己第三方库下面的sophus文件
include_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty
  )
  
message("---------------- ${CATKIN_DEVEL_PREFIX}")
include_directories(
  ${CATKIN_DEVEL_PREFIX}/include  # 手动添加devel路径
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

# add_executable(fusion_slam 
#   app/fusion_slam_node.cc
#   src/SystemConfig.cc
#   src/System.cc
#   src/common/lidar_model.cc
#   src/lidar_process.cc
#   src/static_imu_init.cc
#   src/lio-ieskf/ieskf.cc
#   src/imu_propagator.cc
#   src/odom_matcher/inc_ndt.cc
#   src/odom_matcher/odom_matcher.cc
#   src/odom_matcher/ndt_odom_matcher.cc
# )
# target_link_libraries(fusion_slam 
#   ${catkin_LIBRARIES}
#   spdlog::spdlog 
#   # $<$<BOOL:${MINGW}>:ws2_32>
#   ${PCL_LIBRARIES}

#   ${YAML_CPP_LIBRARIES}
#   ${Boost_LIBRARIES}
#   pthread
#   tbb
# )

add_executable(lio_node 
  src/lio_node.cc
  src/static_imu_init.cc
  src/fastlio_odom/fastlio_ieskf.cc
  src/fastlio_odom/imu_process.cc
  src/fastlio_odom/lidar_process.cc
  src/fastlio_odom/fastlio_odom.cc
  src/ikdtree/ikd_Tree.cc

)
target_link_libraries(lio_node 
  ${catkin_LIBRARIES}
  spdlog::spdlog 
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
  pthread
  tbb
)

