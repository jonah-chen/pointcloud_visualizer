
#include "renderer.hpp"
#include "points.hpp"
#include "point_ops.hpp"
#include "input.hpp"
#include "hud.hpp"

#include <string>
#include <iostream>
#include <future>
#include <chrono>
#include <unordered_map>

static unsigned int age = 0;

std::unordered_map<std::string, char*> parse_args(int argc, char **argv)
{
    std::unordered_map<std::string, char*> args;
    if (argc == 2 && strcmp("--help", argv[1]) == 0)
    {
        std::cout << "Usage: ./renderer <input ply/pcd> [OPTIONS]\n"
                     "Options:\n"
                     "  --windowed: run in windowed mode\n"
                     "  --yz      : swap yz coordinates in input file\n";
        exit(EXIT_SUCCESS);
    }

    args["pointcloud"] = argv[1];

    for (int i = 2; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg.find("--") == 0)
        {
            std::string key = arg.substr(2);
            args[key] = nullptr;
            if (i + 1 < argc)
            {
                std::string value = argv[i + 1];
                if (value.find("--") == 0)
                    continue;
                args[key] = argv[i + 1];
            }
        }
    }
    for (auto &[key, value] : args)
        std::cout << key << ": " << value << "\n";
    return args;
}

int main(int argc, char** argv)
{
    auto args = parse_args(argc, argv);

    GLFWwindow *window;
    Renderer renderer(&window, args.find("windowed") == args.end());
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
    PointCloud vertices = load(args["pointcloud"], camera.pos(), 
        args.find("yz") == args.end());

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

