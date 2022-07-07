cmake_minimum_required(VERSION 3.0.2)
project(assistant)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -std=c++14 -O1")

set(commonPath ../common)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
  visualization_msgs
  interactive_markers
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)

catkin_package(
  DEPENDS PCL EIGEN3
)

include_directories(
  include 
  ${commonPath}/include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

# add_executable(ndt src/ndt.cpp)
# add_dependencies(ndt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ndt  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  yaml-cpp)

# add_executable(keyPointSave src/keyPointSave.cpp)
# add_dependencies(keyPointSave ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(keyPointSave
#   ${catkin_LIBRARIES}  ${PCL_LIBRARIES} 
# )

# add_executable(segmentationSave
#   src/segmentationSave.cpp
# )
# add_dependencies(segmentationSave ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(segmentationSave
#   ${catkin_LIBRARIES}  ${PCL_LIBRARIES} 
# )

##################点云处理##########################

# add_executable(pointCloudProcess  src/pointCloudProcess.cpp )
# add_dependencies(pointCloudProcess ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(pointCloudProcess  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} yaml-cpp)

add_executable(pointPosition  src/pointPosition.cpp )
add_dependencies(pointPosition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointPosition  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )
############################################