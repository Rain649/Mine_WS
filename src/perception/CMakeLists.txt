cmake_minimum_required(VERSION 3.0.2)
project(perception)


# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "RELEASE")

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} ${OpenMP_CXX_FLAGS} -std=c++14 -O2 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} ${OpenMP_CXX_FLAGS} -std=c++14 -O2 -Wall")


set(commonPath ../common)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation 
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

add_service_files(
    FILES
    mineServer.srv
 )

generate_dynamic_reconfigure_options(
  cfg/intersectionDetection.cfg
  cfg/laneDetection.cfg
  cfg/topoMap.cfg
  cfg/lidarCloudProcess.cfg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)
catkin_package(
  DEPENDS roscpp std_msgs PCL EIGEN3 message_generation
)

include_directories(
  include 
  ${commonPath}/include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)


add_executable(lidarCloudProcess  src/lidarCloudProcess.cpp)
add_dependencies(lidarCloudProcess ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidarCloudProcess  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} )

add_executable(intersectionDetection  src/intersectionDetection.cpp)
add_dependencies(intersectionDetection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(intersectionDetection  ${catkin_LIBRARIES}  ${PCL_LIBRARIES})

add_executable(laneDetection  src/laneDetection.cpp ${commonPath}/src/polyfit.cpp)
add_dependencies(laneDetection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(laneDetection  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}  OpenMP::OpenMP_CXX)

add_executable(navigation  src/navigation.cpp src/intersectionLocation.cpp ${commonPath}/src/topoMap.cpp)
add_dependencies(navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} perception_gencpp)
target_link_libraries(navigation  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} yaml-cpp OpenMP::OpenMP_CXX)

add_executable(service src/service.cpp)
target_link_libraries(service ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(service perception_gencpp)

add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(client perception_gencpp)