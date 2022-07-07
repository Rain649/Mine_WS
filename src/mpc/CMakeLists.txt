cmake_minimum_required (VERSION 3.5)
project(mpc)

add_definitions(-std=c++14 -O3)

# SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_BUILD_TYPE "RELEASE")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} ${OpenMP_CXX_FLAGS} -O2 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} ${OpenMP_CXX_FLAGS} -O2 -Wall")

set(commonPath ../common)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_ros
    gazebo_ros
    pcl_conversions
    std_msgs
    gazebo_msgs
    sensor_msgs
    dynamic_reconfigure
    visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)
find_package(PCL REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/mpc.cfg
)

catkin_package(
  DEPENDS PCL EIGEN3 roscpp gazebo_ros dynamic_reconfigure
)

include_directories(
    include
    ${commonPath}/include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR} 
    ${GAZEBO_INCLUDE_DIRS}
)

aux_source_directory(src sources)
link_directories(
  /usr/local/lib
  /usr/local/bin
  ${GAZEBO_LIBRARY_DIRS}
)


add_executable(mpc ${commonPath}/src/polyfit.cpp ${commonPath}/src/topoMap.cpp ${sources})
add_dependencies(mpc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc  ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} ${GAZEBO_LIBRARIES} yaml-cpp ipopt)
