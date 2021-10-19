cmake_minimum_required(VERSION 3.0.2)
project(perception)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O1")

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
  cfg/simuSegSave.cfg
)

find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

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

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)


# add_executable(keyPointSave
#   src/keyPointSave.cpp
# )
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

# add_executable(topoMap
#   src/topoMap.cpp
# )
# add_dependencies(topoMap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(topoMap
#   ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  yaml-cpp
# )

# add_executable(ndt src/ndt.cpp)
# add_dependencies(ndt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ndt  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  yaml-cpp)

# ${OpenCV_LIBS}

############################################

# add_executable(intersection  src/intersection.cpp)
# add_dependencies(intersection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(intersection  ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

# add_executable(laneDetection  src/laneDetection.cpp)
# add_dependencies(laneDetection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(laneDetection  ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

# add_executable(simuSegSave  src/simuSegSave.cpp)
# add_dependencies(simuSegSave ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(simuSegSave  ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

add_executable(main  src/main.cpp src/intersectionLocation.cpp src/topoMap.cpp include/intersectionLocation.h include/topoMap.h)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} yaml-cpp)
