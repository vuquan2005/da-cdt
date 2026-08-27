cmake_minimum_required(VERSION 3.8)
project(robot0_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

add_library(robot0_planar_velocity_control SHARED
  src/PlanarVelocityControl.cpp
)

target_link_libraries(robot0_planar_velocity_control
  PUBLIC
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::ignition-plugin1
    ignition-transport11::ignition-transport11
    ignition-msgs8::ignition-msgs8
)

install(
  TARGETS robot0_planar_velocity_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
