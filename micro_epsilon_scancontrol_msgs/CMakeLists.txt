cmake_minimum_required(VERSION 2.8.3)
project(micro_epsilon_scancontrol_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_service_files(DIRECTORY srv
  FILES 
  GetAvailableResolutions.srv
  SetResolution.srv
  GetResolution.srv
  SetFeature.srv
  GetFeature.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)
