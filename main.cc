
#include "renderer.hpp"
#include "camera.hpp"
#include "points.hpp"
#include "point_ops.hpp"
#include "input.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <future>
#include <chrono>

static unsigned int age = 0;
static float point_size_1m = 20.0f;
static float max_point_size_dist = 60.0f;

void position_display(Camera &camera);

int main(int, char**)
{
    Camera camera(
        glm::vec3(0.0f, 0.0f, 0.0f),    // position
        glm::vec3(0.0f, 0.0f, -1.0f),   // forward
        glm::vec3(0.0f, 1.0f, 0.0f),    // up
        1.2f,                           // fov
        4.0f / 3.0f,                    // aspect
        0.1f,                           // near plane
        10.0f,                          // far plane
        0.1f                            // ground plane
    );

    user_inputs inputs, prev_inputs;
    PointCloud vertices = load("/home/hina/Downloads/1.ply", camera.pos(), true);

    GLFWwindow *window;
    Renderer renderer(&window);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Setup Dear ImGui style
    // ImGui::StyleColorsDark();
    ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    std::future<PointCloud> buf_sorted;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        prev_inputs = inputs;
        glfwPollEvents();
        inputs = user_inputs::fetch(window);

        execute_movement(camera, inputs, prev_inputs, Renderer::FPS);
        
        if (K_ESC(inputs) && !K_ESC(prev_inputs))
        {
            // find cursor mode
            auto cursor_mode = glfwGetInputMode(window, GLFW_CURSOR);
            if (cursor_mode == GLFW_CURSOR_DISABLED)
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            else
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        }

        if (K_R(inputs))
        {
            update(vertices, camera.pos());
            resort(vertices);
            age = 0;
        }
        else
        {
            age++;
            if (!buf_sorted.valid())
            {
                update(vertices, camera.pos());
                buf_sorted = std::async(std::launch::async, update_resort_async, vertices, camera.pos());
            }
            else if (buf_sorted.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                vertices = buf_sorted.get();
                age = 0;
            }
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        position_display(camera);

        // Rendering
        ImGui::Render();
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        renderer.draw(vertices.data(), vertices.size(), camera.view_proj(), 
            point_size_1m, max_point_size_dist / 100.f);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    return 0;
}

void position_display(Camera &camera)
{
    ImGui::Begin("Info");
    ImGui::Text("Position  : %.2f/%.2f/%.2f | Age: %d frames", 
        camera.pos().x, camera.pos().y, camera.pos().z, age);
    // ground height slider
    ImGui::SliderFloat("Ground", &camera.ground_level, -3.f, 3.f);
    ImGui::SliderFloat("Point Size @100cm", &point_size_1m, 1.f, 50.f);
    ImGui::SliderFloat("Max Point Size Distance (cm)" , &max_point_size_dist, 10.f, 200.f);
    ImGui::Text("Facing    : %.4f/%.4f/%.4f", 
        camera.fwd().x, camera.fwd().y, camera.fwd().z);
    ImGui::Text("Frame time: %.3f ms/frame (%.1f FPS)", 
        1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::Separator();
    ImGui::Text("Controls");
    ImGui::Text("WASD   - move");
    ImGui::Text("Space  - up");
    ImGui::Text("LShift - down");
    ImGui::Text("LCtrl  - sprint");
    ImGui::Text("R      - force re-sort points");
    ImGui::Text("Z/X    - move ground level");
    ImGui::End();
}