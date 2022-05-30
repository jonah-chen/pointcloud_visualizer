#include "renderer.hpp"
#include "camera.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char *argv[])
{
    GLFWwindow *window;
    Renderer renderer(&window);
    // set camera matrices
    glm::mat4 view_proj = glm::mat4(1.0f);

    // draw one red point at 0,0,0
    float vertices[] = {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 0.5f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.5f, 0.5f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, -0.5f,
        0.0f, 0.0f, 0.0f
    };


    while (!glfwWindowShouldClose(window))
    {
        // input
        glfwPollEvents();
        // render
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        renderer.draw(vertices, 4, view_proj);
        // swap buffers
        glfwSwapBuffers(window);
    }

    return 0;
}