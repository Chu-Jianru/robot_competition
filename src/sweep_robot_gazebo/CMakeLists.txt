cmake_minimum_required(VERSION 3.8)
project(sweep_robot_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)

add_library(keep_upright SHARED src/keep_upright.cpp)
target_include_directories(keep_upright PUBLIC ${gazebo_dev_INCLUDE_DIRS})
target_link_libraries(keep_upright ${gazebo_dev_LIBRARIES})

install(TARGETS keep_upright
  LIBRARY DESTINATION lib
)

ament_package()
