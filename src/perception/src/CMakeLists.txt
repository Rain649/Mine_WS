cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O1")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -w -Ofast")

project(Point_Cloud_Registration)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(HEADER_DIR /home/lsj/dev/Mine_WS/src/perception/include)
set(HEADER_FILES ${HEADER_DIR}/topoMap.h)

set(SOURCE_DIR ${PROJECT_SOURCE_DIR})

set(SOURCE_FILES ${SOURCE_DIR}/main.cpp ${SOURCE_DIR}/topoMap.cpp)

find_package(PCL 1.7 REQUIRED COMPONENTS common io kdtree features visualization recognition)

option(WITH_OPENMP "WITH_OPENMP" ON)
if(WITH_OPENMP)
	find_package(OpenMP)
	if(OPENMP_FOUND)
		message("OpenMP found")
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
	endif()
endif()

if(PCL_FOUND)
	message("PCL found")

	include_directories(${PCL_INCLUDE_DIRS})
	include_directories(${HEADER_DIR})
    # link_directories("/home/lsj/dev/Mine_WS/src/registration/test/lib")
	
	add_definitions(${PCL_DEFINITIONS})

	add_executable(main ${SOURCE_FILES} ${HEADER_FILES})

    target_link_libraries(main ${PCL_LIBRARIES} yaml-cpp)
endif()