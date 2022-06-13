
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
    HUD(GLFWwindow *window, Camera &camera, std::vector<Mask> &masks);
    ~HUD();

    /**
     * Configure the HUD.
     * When inheriting, the HUD is always configured with
     *  - info
     *  (seperator)
     *  - controls
     *  (seperate window)
     *  - masks (if any)
     *  (seperate window)
     *  - extra
     */
    void configure();

    /**
     * Render the HUD. Should be called after all other draws on this frame.
     */
    void render();

protected:
    Camera &camera_;
    std::vector<Mask> &masks_;
    float default_ground_level_;

    /**
     * Config for different parts of the HUD.
     */
    virtual void C_info();
    virtual void C_controls();
    virtual void C_masks();
    virtual void C_extra() {}
};

class PointHUD : public HUD
{
public:
    PointHUD(GLFWwindow *window, Camera &camera, std::vector<Mask> &masks,
             PointRenderer &renderer);
    
private:
    void C_controls() override;
    float *point_size_1m_ptr_, *max_point_size_dist_ptr_;
};

class MeshHUD : public HUD
{
public:
    MeshHUD(GLFWwindow *window, Camera &camera, std::vector<Mask> &masks,
            MeshRenderer &renderer);
    
private:
    void C_controls() override;
    float *lightColor_ptr_, *lightPos_ptr_;
    float *ambientIntensity_ptr_, *diffuseIntensity_ptr_, *specIntensity_ptr_;
};