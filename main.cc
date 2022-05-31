#include "renderer.hpp"
#include "camera.hpp"
#include "points.hpp"
#include "input.hpp"

int main(int argc, char *argv[])
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

    inputs.fetch(window);

    std::cout << vertices.size() << std::endl;
    

    while (!glfwWindowShouldClose(window))
    {
        // get time
        double time = glfwGetTime();

        // input
        prev_inputs = inputs;
        glfwPollEvents();
        inputs = user_inputs::fetch(window);

        execute_movement(camera, inputs, prev_inputs, Renderer::FPS);

        // render
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        update(vertices, camera.pos());
        renderer.draw(vertices.data(), vertices.size(), camera.view_proj());

        // swap buffers
        glfwSwapBuffers(window);

        std::cout << "FPS: " << 1.0 / (glfwGetTime() - time) << std::endl;
    }

    return 0;
}