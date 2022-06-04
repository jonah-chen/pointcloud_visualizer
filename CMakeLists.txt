cmake_minimum_required(VERSION 3.16 FATAL_ERROR)
project(Pointcloud_Visualizer)

set(CMAKE_CXX_STANDARD 20)

find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenMP REQUIRED)
find_package(fmt REQUIRED)


set(SRC_DIR src)
set(INCLUDE_DIR include)
set(LIB_DIR lib)

set(Point_Dependencies OpenGL
                       glfw
                       GLEW
                       ${PCL_LIBRARIES}
                       ImGui_GLFW
                       OpenMP::OpenMP_CXX)

set(Point_Src ${SRC_DIR}/camera.cc
              ${SRC_DIR}/points.cc
              ${SRC_DIR}/renderer.cc
              ${SRC_DIR}/input.cc
              ${SRC_DIR}/point_ops.cc
              ${SRC_DIR}/hud.cc
              ${SRC_DIR}/parser.cc)

include_directories(${SRC_DIR} 
                    ${INCLUDE_DIR}
                    ${INCLUDE_DIR}/imgui
                    ${PCL_INCLUDE_DIRS}
                    ${OpenGL_INCLUDE_DIRS}
                    ${glfw3_INCLUDE_DIRS}
                    ${GLEW_INCLUDE_DIRS})
link_directories(${LIB_DIR})

add_library(ImGui_GLFW STATIC 
            ${INCLUDE_DIR}/imgui/imgui_impl_glfw.cpp
            ${INCLUDE_DIR}/imgui/imgui_impl_opengl3.cpp
            ${INCLUDE_DIR}/imgui/imgui.cpp
            ${INCLUDE_DIR}/imgui/imgui_tables.cpp
            ${INCLUDE_DIR}/imgui/imgui_widgets.cpp
            ${INCLUDE_DIR}/imgui/imgui_draw.cpp)
target_link_libraries(ImGui_GLFW ${glfw3_LIBRARIES})


add_executable(Pointcloud_Visualizer ${SRC_DIR}/pointcloud_visualizer.cc ${Point_Src})
target_link_libraries(Pointcloud_Visualizer PUBLIC ${Point_Dependencies})

add_executable(Meshed_Visualizer ${SRC_DIR}/meshed_visualizer.cc 
                                ${Point_Src}
                                ${SRC_DIR}/mesh.cc)
target_link_libraries(Meshed_Visualizer PUBLIC ${Point_Dependencies} Open3D)

add_executable(BPA utils/bpa.cc)
target_link_libraries(BPA PUBLIC Open3D pthread)

add_executable(Poission utils/poission.cc)
target_link_libraries(Poission PUBLIC Open3D pthread)