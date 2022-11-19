cmake_minimum_required(VERSION 3.21.0)
project(Simulator)

if(WIN32)
    find_package(mujoco PATHS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/mujoco)
    find_package(glfw3 PATHS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glfw3)
elseif(UNIX AND NOT APPLE)
    find_package(mujoco REQUIRED)
    find_package(glfw3 REQUIRED)
endif()

link_libraries(mujoco::mujoco)

set(CMAKE_CONFIGURATION_TYPES Release)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR})

set(LIB_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Share/src)
set(LIB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Share/include)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(${INCLUDE_DIR} ${LIB_INCLUDE_DIR})

file(GLOB SOURCES ${SOURCE_DIR}/*.cpp ${INCLUDE_DIR}/*.h ${LIB_SOURCE_DIR}/*.cpp ${LIB_INCLUDE_DIR}/*.h)

add_executable(Simulator ${SOURCES})
target_link_libraries(Simulator ${MUJOCO_LIBRARIES} glfw)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/model/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/model)

if (WIN32)
    add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/mujoco/bin/mujoco.dll
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)
endif()

if (WIN32)
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Simulator)
endif()
