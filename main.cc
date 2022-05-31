
#include "renderer.hpp"
#include "camera.hpp"
#include "points.hpp"
#include "input.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

int main(int, char**)
{
    Camera camera(
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, -1.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        1.4f,
        4.0f / 3.0f,
        0.1f,
        10.0f,
        0.1f
    );

    user_inputs inputs, prev_inputs;
    auto vertices = load("sample.ply", camera.pos(), false);

    GLFWwindow *window;
    Renderer renderer(&window);
    // set camera matrices

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        prev_inputs = inputs;
        glfwPollEvents();
        inputs = user_inputs::fetch(window);

        execute_movement(camera, inputs, prev_inputs, Renderer::FPS);
        // update(vertices, camera.pos());


        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
        {
            ImGui::Begin("Stats");
            ImGui::Text("Position : %f/%f/%f", camera.pos().x, camera.pos().y, camera.pos().z);
            ImGui::Text("Facing   : %f/%f/%f", camera.fwd().x, camera.fwd().y, camera.fwd().z);
            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::Separator();
            ImGui::Text("Controls");
            ImGui::Text("WASD   - move");
            ImGui::Text("Space  - up");
            ImGui::Text("LShift - down");
            
            ImGui::End();
        }


        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        renderer.draw(vertices.data(), vertices.size(), camera.view_proj());
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    return 0;
}


// #include "renderer.hpp"
// #include "camera.hpp"
// #include "points.hpp"
// #include "input.hpp"
// #include <imgui.h>
// #include "imgui_impl_glfw.h"
// #include "imgui_impl_opengl3.h"

// int main(int argc, char *argv[])
// {
//     Camera camera(
//         glm::vec3(0.0f, 0.0f, 0.0f),
//         glm::vec3(0.0f, 0.0f, -1.0f),
//         glm::vec3(0.0f, 1.0f, 0.0f),
//         1.4f,
//         4.0f / 3.0f,
//         0.1f,
//         10.0f,
//         0.1f
//     );

//     user_inputs inputs, prev_inputs;
    
//     auto vertices = load("sample.ply", camera.pos(), false);

//     GLFWwindow *window;
//     Renderer renderer(&window);
//     // set camera matrices

//     inputs.fetch(window);

//     IMGUI_CHECKVERSION();
//     ImGui::CreateContext();
//     ImGuiIO& io = ImGui::GetIO();
//     (void)io;

//     ImGui::StyleColorsDark();
//     ImGui_ImplGlfw_InitForOpenGL(window, true);
//     ImGui_ImplOpenGL3_Init("#version 130");

//     bool show_demo_window = true;
//     bool show_another_window = false;
//     ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);


//     while (!glfwWindowShouldClose(window))
//     {
//         // get time
//         double time = glfwGetTime();

//         // input
//         prev_inputs = inputs;
//         glfwPollEvents();
//         inputs = user_inputs::fetch(window);

//         ImGui_ImplOpenGL3_NewFrame();
//         ImGui_ImplGlfw_NewFrame();
//         ImGui::NewFrame();
//         if (show_demo_window)
//             ImGui::ShowDemoWindow(&show_demo_window);

//         {
//             static float f = 0.0f;
//             static int counter = 0;

//             ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

//             ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)
//             ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state
//             ImGui::Checkbox("Another Window", &show_another_window);

//             ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
//             ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

//             if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
//                 counter++;
//             ImGui::SameLine();
//             ImGui::Text("counter = %d", counter);

//             ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
//             ImGui::End();
//         }



//         execute_movement(camera, inputs, prev_inputs, Renderer::FPS);

//         // render
//         glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
//         glClear(GL_COLOR_BUFFER_BIT);

//         update(vertices, camera.pos());
//         renderer.draw(vertices.data(), vertices.size(), camera.view_proj());
//         ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

//         // swap buffers
//         glfwSwapBuffers(window);

//         std::cout << "FPS: " << 1.0 / (glfwGetTime() - time) << std::endl;
//     }


//     ImGui_ImplOpenGL3_Shutdown();
//     ImGui_ImplGlfw_Shutdown();
//     ImGui::DestroyContext();

//     return 0;
// }