cmake_minimum_required(VERSION 2.8.3)
project(micro_epsilon_scancontrol_driver)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros nodelet micro_epsilon_scancontrol_msgs)
find_package(PkgConfig REQUIRED)

pkg_check_modules(scancontrol REQUIRED glib-2.0 aravis-0.6 mescan llt)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp pcl_ros nodelet micro_epsilon_scancontrol_msgs
  DEPENDS scancontrol
)

include_directories(include ${catkin_INCLUDE_DIRS} ${scancontrol_INCLUDE_DIRS})

add_executable(driver_node src/node.cpp src/driver.cpp)
target_link_libraries(driver_node ${catkin_LIBRARIES} ${scancontrol_LIBRARIES})

add_library(driver_nodelet src/nodelet.cpp src/driver.cpp)
target_link_libraries(driver_nodelet ${catkin_LIBRARIES} ${scancontrol_LIBRARIES})
