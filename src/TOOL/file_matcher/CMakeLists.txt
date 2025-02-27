cmake_minimum_required(VERSION 3.0.2)
project(file_matcher)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  # pcl_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

# find_package(PCL REQUIRED)
find_package(PythonLibs REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package(
  INCLUDE_DIRS include
  # DEPENDS PCL

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_executable(result_of_btc_ndt_compare src/resultOfBtcNdtCompare.cpp src/fileMatcher.cpp)

target_link_libraries(result_of_btc_ndt_compare 
  ${catkin_LIBRARIES} 
  ${PYTHON_LIBRARIES}
  )