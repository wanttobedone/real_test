cmake_minimum_required(VERSION 3.0.2)
project(real_test)

## 编译选项
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

## 查找catkin包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## Catkin包配置
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    mavros_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

## 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 可执行文件：测试节点1 - 基础起飞悬停降落
add_executable(test_node1 
  src/test_node1.cpp
)

target_link_libraries(test_node1
  ${catkin_LIBRARIES}
)

## 安装规则
install(TARGETS test_node1
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)