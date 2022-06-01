cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(Pointcloud_Visualizer)

set(CMAKE_CXX_STANDARD 20)

find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(ImGui REQUIRED)
find_package(OpenMP REQUIRED)

set(SRC_DIR src)
set(INCLUDE_DIR include)

include_directories(${SRC_DIR} 
                    ${INCLUDE_DIR}
                    ${PCL_INCLUDE_DIRS}
                    ${OpenGL_INCLUDE_DIRS}
                    ${glfw3_INCLUDE_DIRS}
                    ${GLEW_INCLUDE_DIRS}
                    ${ImGui_INCLUDE_DIRS})

add_library(ImGui_GLFW STATIC ${INCLUDE_DIR}/imgui_impl_glfw.cpp ${INCLUDE_DIR}/imgui_impl_opengl3.cpp)
target_link_libraries(ImGui_GLFW ${glfw3_LIBRARIES} imgui)

add_executable(Pointcloud_Visualizer ${SRC_DIR}/main.cc 
                ${SRC_DIR}/camera.cc
                ${SRC_DIR}/points.cc
                ${SRC_DIR}/renderer.cc
                ${SRC_DIR}/input.cc
                ${SRC_DIR}/point_ops.cc
                ${SRC_DIR}/hud.cc)
target_link_libraries(Pointcloud_Visualizer PUBLIC 
                        OpenGL
                        glfw
                        GLEW
                        ${PCL_LIBRARIES}
                        ImGui_GLFW
                        OpenMP::OpenMP_CXX)
