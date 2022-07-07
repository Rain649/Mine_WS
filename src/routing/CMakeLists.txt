cmake_minimum_required(VERSION 3.5)
project(routing)

set(commonPath ../common)

add_definitions(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(routing src/main.cpp)
add_dependencies(routing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(routing ${catkin_LIBRARIES} yaml-cpp)
