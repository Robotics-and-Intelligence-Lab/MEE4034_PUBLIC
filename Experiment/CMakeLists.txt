cmake_minimum_required(VERSION 3.14.0)
project(Experiment)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(LIB_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Share/src)
set(LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Share/include)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(${INCLUDE_DIR} ${LIB_INCLUDE_DIR})

file(GLOB SOURCES ${SOURCE_DIR}/*.cpp ${INCLUDE_DIR}/*.h ${LIB_SOURCE_DIR}/*.cpp ${LIB_INCLUDE_DIR}/*.h)

if(UNIX AND NOT APPLE)
    add_executable(Experiment ${SOURCES})
    target_link_libraries(Experiment -lEposCmd -lrt)
else()
    add_executable(Experiment ${SOURCE_DIR}/main.cpp)
endif()
