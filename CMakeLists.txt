cmake_minimum_required(VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeFiles/modules/")

PROJECT(openal++)

INCLUDE (CheckIncludeFiles)

# build type
set(CMAKE_BUILD_TYPE Debug)

# compile option (Debug) 
set(CMAKE_CXX_FLAGS_DEBUG "-g -m32")
set(CMAKE_C_FLAGS_DEBUG "-g -m32")

# compile option (Release)
set(CMAKE_CXX_FLAGS_RELEASE "-W -Wall -O3")
set(CMAKE_C_FLAGS_RELEASE "-W -Wall -O3")

#if you don't want the full compiler output, remove the following line
set(CMAKE_VERBOSE_MAKEFILE OFF)

set(Boost_USE_MULTITHREADED ON)
find_package(Boost 1.42.0 COMPONENTS thread REQUIRED )
find_package(OpenAL REQUIRED )
find_package(TBB REQUIRED )

INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_BINARY_DIR}/include ${Boost_INCLUDE_DIRS} ${OPENAL_INCLUDE_PATH} ${TBB_INCLUDE_DIR} )

subdirs( include src )
