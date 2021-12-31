cmake_minimum_required(VERSION 3.0.2)
project(routing_delta)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(main_routing 
#   src/main_routing.cc
#   src/core_routing.cc
#   src/topoMap.cpp
# )

# add_dependencies(main_routing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(main_routing
#   ${catkin_LIBRARIES}  yaml-cpp
# )
