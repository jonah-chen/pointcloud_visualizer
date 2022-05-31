
#pragma once

#include "renderer.hpp"
#include "camera.hpp"
#include "point_ops.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

class HUD
{
public:
    HUD(GLFWwindow *window, Camera &camera, unsigned int &age, 
        Renderer &renderer, std::vector<Mask> &masks);
    ~HUD();
    void configure();
    void render();

private:
    Camera &camera_;
    float *point_size_1m_ptr_, *max_point_size_dist_ptr_;
    unsigned int &age_;
    std::vector<Mask> &masks_;
};
