cmake_minimum_required(VERSION 2.8.3)
project(rt_dynamixel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  realtime_tools
  std_msgs
  rt_dynamixel_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rt_dynamixel
#  CATKIN_DEPENDS roscpp rosrt
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/rt_dynamixel/
)

add_library(rt_serial_port SHARED src/rt_serial_port.cpp)
add_library(rt_dynamixel_pro SHARED src/rt_dynamixel_pro.cpp)
add_library(rt_ros_service STATIC src/rt_ros_service.cpp)
add_library(dxl_lists STATIC src/dxl_lists.cpp)

add_executable(rt_dynamixel_node src/rt_dynamixel_node.cpp)

target_link_libraries(rt_serial_port
#  ${catkin_LIBRARIES}
  -lpthread
)
target_link_libraries(rt_dynamixel_pro
#  ${catkin_LIBRARIES}
  -lpthread

)

target_link_libraries(rt_ros_service
  ${catkin_LIBRARIES}
  -lpthread
  rt_serial_port
  rt_dynamixel_pro
)

target_link_libraries(rt_dynamixel_node
  ${catkin_LIBRARIES}
  -lpthread
  rt_serial_port
  rt_dynamixel_pro
  rt_ros_service
  dxl_lists
)

set_property(TARGET rt_dynamixel_pro PROPERTY C_STANDARD 99)
set(CMAKE_BUILD_TYPE Debug)


