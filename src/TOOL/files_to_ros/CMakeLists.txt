cmake_minimum_required(VERSION 2.8.3)
project(files_to_ros)


set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL REQUIRED QUIET)
find_package(OpenMP REQUIRED)



catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL

  CATKIN_DEPENDS
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  # message_runtime # for new msg
  # message_generation # for new msg
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

# exe
add_executable(${PROJECT_NAME}_files_to_ros
 src/files_to_ros.cpp 
 )
add_dependencies(${PROJECT_NAME}_files_to_ros ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME}_files_to_ros PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_files_to_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_CXX_FLAGS}  stdc++fs)

add_executable(${PROJECT_NAME}_load_pcd_to_view
 src/load_pcd_to_view.cpp 
 )
add_dependencies(${PROJECT_NAME}_load_pcd_to_view ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME}_load_pcd_to_view PRIVATE)
target_link_libraries(${PROJECT_NAME}_load_pcd_to_view ${catkin_LIBRARIES} ${PCL_LIBRARIES}  stdc++fs)