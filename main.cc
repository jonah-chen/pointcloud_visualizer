
#include "renderer.hpp"
#include "points.hpp"
#include "point_ops.hpp"
#include "input.hpp"
#include "hud.hpp"

#include <future>
#include <chrono>

static unsigned int age = 0;



int main(int argc, char** argv)
{
    GLFWwindow *window;
    Renderer renderer(&window);
    Camera camera(
        glm::vec3(0.0f, 0.0f, 0.0f),                            // position
        glm::vec3(0.0f, 0.0f, -1.0f),                           // forward
        glm::vec3(0.0f, 1.0f, 0.0f),                            // up
        1.2f,                                                   // fov
        (float) renderer.width() / (float) renderer.height(),   // aspect
        0.1f,                                                   // near plane
        20.0f,                                                  // far plane
        0.0f                                                    // ground plane
    );
    HUD hud(window, camera, age, renderer);

    std::future<PointCloud> buf_sorted;
    user_inputs inputs, prev_inputs;
    PointCloud vertices = load("/home/hina/Downloads/1.ply", camera.pos(), true);


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

        hud.configure();

        // Rendering
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        renderer.draw(vertices.data(), vertices.size(), camera);
        hud.render();

        glfwSwapBuffers(window);
    }

    return 0;
}

