cmake_minimum_required(VERSION 3.0.2)
project(control_underground_delta)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_path_follow
  src/main_path_follow.cpp
  src/sub_path_follow.cpp
  src/core_path_follow.cpp
  src/pub_path_follow.cpp
)

add_executable(cmd_sim
  src/cmd_sim.cpp
)

add_executable(odom_sim
  src/odom_sim.cpp
)

add_dependencies(main_path_follow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(cmd_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(odom_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(main_path_follow
  ${catkin_LIBRARIES}
)

target_link_libraries(cmd_sim
  ${catkin_LIBRARIES}
)

target_link_libraries(odom_sim
  ${catkin_LIBRARIES}
)
