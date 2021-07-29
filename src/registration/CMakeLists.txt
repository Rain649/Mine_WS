cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

#同时生成动态库和静态库
project(registration)
aux_source_directory(src DIR_SRCS)
list(REMOVE_ITEM DIR_SRCS "src/main.cpp")
# include_directories(/usr/include/eigen3/eigen)
find_package(PCL 1.7 REQUIRED COMPONENTS common io kdtree features visualization recognition)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

#生成动态库
ADD_LIBRARY(registration SHARED ${DIR_SRCS})
#指定动态库版本号
# SET_TARGET_PROPERTIES(registration PROPERTIES VERSION 1.0 SOVERSION 1)
#
INSTALL(FILES registration.h DESTINATION include)

# ADD_LIBRARY(registration_s STATIC ${DIR_SRCS})
# SET_TARGET_PROPERTIES(registration_s PROPERTIES OUTPUT_NAME "registration")

# INSTALL(TARGETS registration registration_s
#           LIBRARY DESTINATION lib 
#           ARCHIVE DESTINATION lib)
