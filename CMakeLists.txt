cmake_minimum_required(VERSION 3.16)
project(super_lio VERSION 2.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
add_definitions(-DROOT=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(livox_msg REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(glog REQUIRED)

file(GLOB_RECURSE BASIC_SRC "src/basic/src/*.cpp")
add_library(basic SHARED ${BASIC_SRC})
target_include_directories(basic PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/basic/include>
)
target_link_libraries(basic PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog::glog
  pthread
  gflags
)

file(GLOB_RECURSE LIO_LIB_SRC "src/super_lio/src/lio/*.cpp" "src/super_lio/src/ros/*.cpp")

fins_add_node(${PROJECT_NAME} 
  src/super_lio/src/apps/super_lio_node.cpp
  ${LIO_LIB_SRC}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/super_lio/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/super_lio/3rdparty>
)

fins_link_ros_dependencies(${PROJECT_NAME}
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  basic
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  glog::glog
  gflags
  pthread
  livox_msg::livox_msg
)