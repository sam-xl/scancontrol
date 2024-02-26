cmake_minimum_required(VERSION 3.5)

project(micro_epsilon_scancontrol_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME})
