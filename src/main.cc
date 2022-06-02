
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

    std::future<PointCloud> buf_sorted;
    std::vector<Mask> masks;
    std::vector<bool> masks_applied;
    bool mask_updated;
    const PointCloud og_cloud = load(args["pointcloud"], args.find("yz") == args.end());

    PointCloud cloud = og_cloud; // make a copy of original order for masking purposes
    float ground_level = 0.0f;

    if (args.find("masks") != args.end())
    {
        masks = load_masks(args["masks"]);
        auto floor_mask = std::find_if(masks.begin(), masks.end(), 
        [](const Mask &mask) { return mask.cls == "floor"; });
        if (floor_mask != masks.end())
        {
            auto center = centroid(cloud, *floor_mask);
            ground_level = center.y + Camera::EYE_HEIGHT;
        }
    }
    for (const auto &mask : masks)
        masks_applied.push_back(mask.active);
    
    
    Renderer renderer(args.find("windowed") == args.end());
    Camera camera(
        centroid(og_cloud),                                     // position
        glm::vec3(0.0f, 0.0f, 1.0f),                            // forward
        glm::vec3(0.0f, 1.0f, 0.0f),                            // up
        1.2f,                                                   // fov
        (float) renderer.width() / (float) renderer.height(),   // aspect
        0.1f,                                                   // near plane
        200.0f,                                                 // far plane
        ground_level                                            // ground plane
    );


    HUD hud(renderer.window(), camera, age, renderer, masks);
    user_inputs inputs, prev_inputs;
    bool to_move = false;

    update(cloud, camera.pos());
    resort(cloud);

    // Main loop
    while (!glfwWindowShouldClose(renderer.window()))
    {
        prev_inputs = inputs;
        glfwPollEvents();
        inputs = user_inputs::fetch(renderer.window());

        if (to_move)
            execute_movement(camera, inputs, prev_inputs, ImGui::GetIO().Framerate);
        else
            to_move = true;

        if (LMB(inputs) && !LMB(prev_inputs))
        {
            renderer.enable_cursor();
            to_move = false;
        }
        else if (RMB(inputs) && !RMB(prev_inputs))
            renderer.disable_cursor();        
        if (K_ESC(inputs))
            break;

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
            // apply the masks in the opposite order
            for (auto msk_it = masks.rbegin(); msk_it != masks.rend(); ++msk_it)
                if (msk_it->active)
                    msk_it->apply(cloud);

            buf_sorted = std::async(std::launch::async, update_resort_async, 
                cloud, camera.pos());
            
            to_move = false;
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

        glfwSwapBuffers(renderer.window());
    }
    
    return 0;
}
