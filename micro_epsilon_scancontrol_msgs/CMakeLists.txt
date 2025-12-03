cmake_minimum_required(VERSION 3.5)
project(micro_epsilon_scancontrol_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(srv_files
  "srv/GetAvailableResolutions.srv"
  "srv/GetFeature.srv"
  "srv/GetResolution.srv"
  "srv/SetFeature.srv"
  "srv/SetResolution.srv"
  "srv/GetDuration.srv"
  "srv/SetDuration.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
