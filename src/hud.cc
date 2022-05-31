
#include "hud.hpp"

HUD::HUD(GLFWwindow *window, Camera &camera, unsigned int &age, 
         Renderer &renderer, std::vector<Mask> &masks)
    : camera_(camera), age_(age), masks_(masks)
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

    ImGui::Text("Position  : %.2f/%.2f/%.2f | Age: %d frames", 
        camera_.pos().x, camera_.pos().y, camera_.pos().z, age_);
    ImGui::Text("Facing    : %.4f/%.4f/%.4f", 
        camera_.fwd().x, camera_.fwd().y, camera_.fwd().z);
    ImGui::Text("Frame time: %.3f ms/frame (%.1f FPS)", 
        1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::Separator();

    ImGui::SliderFloat("Ground", &camera_.ground_level, -3.f, 3.f);
    ImGui::SliderFloat("Point Size @1m", point_size_1m_ptr_, 1.f, 64.f, "%.1f", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderFloat("Max Point Size Distance (cm)" , max_point_size_dist_ptr_, 10.f, 200.f);
    ImGui::Separator();

    ImGui::Text("W/A/S/D - move");
    ImGui::Text("Space   - up");
    ImGui::Text("LShift  - down");
    ImGui::Text("LCtrl   - sprint");
    ImGui::Text("R       - force re-sort points");
    ImGui::Text("Esc     - toggle cursor mode");
    ImGui::End();

    ImGui::Begin("Masks");
}

void HUD::render()
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}
