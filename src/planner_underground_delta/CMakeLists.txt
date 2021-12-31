cmake_minimum_required(VERSION 3.0.2)
project(planner_underground_delta)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package()

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# add_executable(main_planner_ug
#   src/main_planner_ug.cc
#   src/core_planner_ug.cc
#   src/core_lka.cc
#   src/sub_db.cpp
#   src/pub_db.cpp
#   src/core_db.cpp
#   src/dubins.cpp
# )

# add_dependencies(main_planner_ug ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(main_planner_ug
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${PCL_LIBRARIES}
# )
