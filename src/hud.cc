
#include "hud.hpp"

HUD::HUD(GLFWwindow *window, Camera &camera, 
         PointRenderer &renderer, std::vector<Mask> &masks)
    : camera_(camera),
        masks_(masks), 
        default_ground_level_(camera.ground_level)
{
    point_size_1m_ptr_ = &(renderer.point_size_1m);
    max_point_size_dist_ptr_ = &(renderer.max_point_size_dist);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");
}

HUD::~HUD()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void HUD::configure()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    ImGui::Begin("Info");

    ImGui::Text("Position  : %.2f/%.2f/%.2f", 
        camera_.pos().x, camera_.pos().y, camera_.pos().z);
    ImGui::Text("Facing    : %.4f/%.4f/%.4f", 
        camera_.fwd().x, camera_.fwd().y, camera_.fwd().z);
    ImGui::Text("Frame time: %.3f ms/frame (%.1f FPS)", 
        1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::Separator();

    ImGui::SliderFloat("Ground", &camera_.ground_level, 
        default_ground_level_ - 2.0f, default_ground_level_ + 2.0f);
    ImGui::SliderFloat("Point Size @1m", point_size_1m_ptr_, 1.f, 64.f, "%.1f", 
        ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Max Point Size Distance (cm)" , 
        max_point_size_dist_ptr_, 10.f, 200.f, "%.1f", ImGuiSliderFlags_Logarithmic);
    ImGui::Separator();

    ImGui::Text("W/A/S/D - move");
    ImGui::Text("Space   - up");
    ImGui::Text("LShift  - down");
    ImGui::Text("LCtrl   - sprint");
    ImGui::Text("LMB/RMB - toggle cursor mode");
    ImGui::Text("Esc     - quit");
    ImGui::End();

    if (!masks_.empty())
    {
        ImGui::Begin("Masks");
        for (int i = 0; i < masks_.size(); ++i)
        {
            auto &mask = masks_[i];
            ImGui::Checkbox((std::to_string(i) + ": ").c_str(), &mask.active);
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(
                mask.color.x, mask.color.y, mask.color.z, 1.0f), "%s", 
                mask.cls.c_str());
        }

        if (ImGui::Button("Show All"))
        {
            for (auto &mask : masks_)
                mask.active = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Hide All"))
        {
            for (auto &mask : masks_)
                mask.active = false;
        }
        ImGui::End();
    }
}

void HUD::render()
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
