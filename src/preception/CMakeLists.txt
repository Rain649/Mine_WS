##此程序由北京理工大学*刘仕杰*编写##
cmake_minimum_required(VERSION 3.0.2)
project(perception)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
#set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.9")
#set(PCL_LIBRARY_DIRS "/usr/local/lib" )
# set(OpenCV_INCLUDE_DIRS "/usr/include")

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs

  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/param.cfg
  cfg/topoMap.cfg
)

find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)

catkin_package(
  DEPENDS PCL EIGEN3
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  #${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR} 
)

include_directories( ${OpenCV_INCLUDE_DIRS} )

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)

add_executable(intersection
  src/intersection.cpp
)
add_dependencies(intersection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(intersection
  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  ${OpenCV_LIBS}
)

add_executable(laneDetection
  src/laneDetection.cpp
)
add_dependencies(laneDetection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laneDetection
  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  ${OpenCV_LIBS}
)

add_executable(keyPointSave
  src/keyPointSave.cpp
)
add_dependencies(keyPointSave ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyPointSave
  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} 
)

add_executable(segmentationSave
  src/segmentationSave.cpp
)
add_dependencies(segmentationSave ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(segmentationSave
  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} 
)

add_executable(pcd_write
  src/pcd_write.cpp
)
add_dependencies(pcd_write ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_write
  ${catkin_LIBRARIES}
)

add_executable(pcd_read
  src/pcd_read.cpp
)
add_dependencies(pcd_read ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_read
  ${catkin_LIBRARIES}
)

add_executable(pcd_visual
  src/pcd_visual.cpp
)
add_dependencies(pcd_visual ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcd_visual
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} 
)

add_executable(topoMap
  src/topoMap.cpp
)
add_dependencies(topoMap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(topoMap
  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  yaml-cpp
)

add_executable(ndt src/ndt.cpp)
add_dependencies(ndt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ndt  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  yaml-cpp)
