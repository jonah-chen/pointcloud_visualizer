cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(Pointcloud_Visualizer)

set(CMAKE_CXX_STANDARD 20)
find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)

set (SRC_DIR src)
# add include directories
include_directories(${SRC_DIR} ${PCL_INCLUDE_DIRS} ${OpenGL_INCLUDE_DIRS} ${glfw3_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})

add_executable(Pointcloud_Visualizer main.cc 
                ${SRC_DIR}/camera.cc
                ${SRC_DIR}/points.cc
                ${SRC_DIR}/renderer.cc)
target_link_libraries(Pointcloud_Visualizer OpenGL glfw GLEW ${PCL_LIBRARIES})

