cmake_minimum_required(VERSION 3.0.2)
project(calcul_pcd_overlap)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
find_package(PCL REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

################ 编译静态库的流程：################
# 导出头文件路径
catkin_package(
  INCLUDE_DIRS include  # 导出头文件路径
  # LIBRARIES calcul_pcd_overlap_lib  # 导出库
)
add_library(calcul_pcd_overlap_lib  # 已编译为静态库lib，注意命名格式，包名：calcul_pcd_overlap  类名：calculPcdOverlap
  src/calculPcdOverlap.cpp
)
target_include_directories(calcul_pcd_overlap_lib PUBLIC
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/calcul_pcd_overlap # 把这个库调用的头文件公开暴露，注意本包的头文件应该放在pkg/include/pkg/**.h处
)
install(DIRECTORY include/${PROJECT_NAME}/    # 安装头文件到devel空间
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
# 安装库
install(TARGETS calcul_pcd_overlap_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
# 如何调用：
# 1.在调用包的package.xml添加依赖。
# <package>
# ...
# <depend>calcul_pcd_overlap</depend>  <!-- 添加依赖 -->
# </package>
# 2.修改调用包的cmakelist.txt
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   ...
#   calcul_pcd_overlap  # 声明依赖 特别注意，“find_package(catkin REQUIRED COMPONENTS”只能写一个，需要做的是把当前这行add到已有的“”当中。
# )
# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )
# target_link_libraries(your_node  # 链接库
#   ${catkin_LIBRARIES}
#   calcul_pcd_overlap_lib
# )
# 3.在调用包引用
#include <calcul_pcd_overlap/calculPcdOverlap.h>  // 使用尖括号<>// 而不是双引号""
# 4.重新编译（可选）
# cd ~/catkin_ws
# catkin clean -y  # 彻底清理旧编译
# catkin build calcul_pcd_overlap  # 先单独编译B包
# catkin build your_package_A      # 再编译A包

################ 编译静态库的流程↑################



add_executable(calcul_pcd_overlap src/calculPcdOverlap.cpp src/compareBtcWithNdt.cpp)

target_link_libraries(calcul_pcd_overlap 
        ${catkin_LIBRARIES} 
        ${PCL_LIBRARIES} 
        )