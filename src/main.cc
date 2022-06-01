
#include "renderer.hpp"
#include "points.hpp"
#include "point_ops.hpp"
#include "input.hpp"
#include "hud.hpp"

#include <iostream>
#include <future>
#include <chrono>
#include <unordered_map>

static unsigned int age = 0;

std::unordered_map<std::string, char*> parse_args(int argc, char **argv)
{
    std::unordered_map<std::string, char*> args;
    if (argc == 1 || argc == 2 && strcmp("--help", argv[1]) == 0)
    {
        std::cout << "Usage: ./renderer <input ply/pcd> [OPTIONS]\n"
                     "Options:\n"
                     "  --windowed : run in windowed mode\n"
                     "  --yz : swap yz coordinates in input file\n"
                     "  --mask <path to mask file> : mask file with format:\n"
                     "      <path to mask> <color hex code> <class>\n"
                     "      ...\n";

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

    std::future<PointCloud> buf_sorted;
    std::vector<Mask> masks;
    std::vector<bool> masks_applied;
    bool mask_updated;
    for (const auto &mask : masks)
        masks_applied.push_back(mask.active);

    user_inputs inputs, prev_inputs;
    PointCloud og_cloud = load(args["pointcloud"], camera.pos(), 
        args.find("yz") == args.end());
    PointCloud cloud = og_cloud; // make a copy of original order for masking purposes
    
    if (args.find("mask") != args.end())
        masks = load_masks(args["mask"]);
      
    HUD hud(window, camera, age, renderer, masks);


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

        mask_updated = false;
        for (size_t i = 0; i < masks.size(); ++i)
        {
            if (masks[i] != masks_applied[i])
            {
                mask_updated = true;
                masks_applied[i] = masks[i].active;
            }
        }
        if (mask_updated)
        {
            cloud = og_cloud;
            for (auto &mask : masks)
            {
                if (mask)
                    mask.apply(cloud);
            }
        }
        if (K_R(inputs) || mask_updated)
        {
            update(cloud, camera.pos());
            resort(cloud);
            age = 0;
        }
        else
        {
            age++;
            if (!buf_sorted.valid())
            {
                buf_sorted = std::async(std::launch::async, update_resort_async,
                    cloud, camera.pos());
            }
            else if (buf_sorted.wait_for(std::chrono::seconds(0)) 
                == std::future_status::ready)
            {
                cloud = buf_sorted.get();
                age = 0;
            }
        }

        hud.configure();

        // Rendering
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        renderer.draw(cloud.data(), cloud.size(), camera);
        hud.render();

        glfwSwapBuffers(window);
    }

    return 0;
}
