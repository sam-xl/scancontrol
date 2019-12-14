cmake_minimum_required(VERSION 2.8.3)
project(scancontrol)

find_package(catkin REQUIRED COMPONENTS roscpp rospy rqt_gui rqt_gui_py pcl_ros nodelet micro_epsilon_scancontrol_msgs)
find_package(PkgConfig REQUIRED)

pkg_check_modules(glib2 REQUIRED glib-2.0)
pkg_check_modules(aravis REQUIRED aravis-0.6)

set(llt_INCLUDE_DIRS /usr/local/include/libllt)
set(llt_LIBRARIES /usr/local/lib/libllt.so)

set(mescan_INCLUDE_DIRS /usr/local/include/libmescan)
set(mescan_LIBRARIES /usr/local/lib/libmescan.so)

#find_package(aravis REQUIRED)
#find_package(glib2 REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy rqt_gui rqt_gui_py pcl_ros nodelet micro_epsilon_scancontrol_msgs
  DEPENDS aravis glib2
)

include_directories(include ${catkin_INCLUDE_DIRS} ${llt_INCLUDE_DIRS} ${mescan_INCLUDE_DIRS} ${glib2_INCLUDE_DIRS} ${aravis_INCLUDE_DIRS})

add_executable(driver_node src/node.cpp src/driver.cpp)
target_link_libraries(driver_node ${catkin_LIBRARIES} ${llt_LIBRARIES} ${mescan_LIBRARIES} ${glib2_LIBRARIES} ${aravis_LIBRARIES})
#add_dependencies(driver_node)

add_library(driver_nodelet src/nodelet.cpp src/driver.cpp)
target_link_libraries(driver_nodelet ${catkin_LIBRARIES} ${llt_LIBRARIES} ${mescan_LIBRARIES} ${glib2_LIBRARIES} ${aravis_LIBRARIES})
#add_dependencies(driver_nodelet)
