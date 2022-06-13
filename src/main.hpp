
#pragma once
#include "mesh.hpp"
#include "renderer.hpp"
#include "camera.hpp"
#include "input.hpp"
#include "point_ops.hpp"
#include "hud.hpp"

#include <functional>
#include <unordered_map>
#include <string>

inline std::unordered_map<std::string, char*> parse_args(int argc, char **argv)
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

template <typename Geometry, typename HUD, typename Renderer>
int _main(int argc, char *argv[], std::function<Geometry(const std::string &, bool)> load_fn)
{
    auto args = parse_args(argc, argv);
    
    std::vector<Mask> masks;
    std::vector<bool> masks_applied;
    bool mask_updated;
    bool mask_invert = args.find("invert") != args.end();
    const Geometry og_cloud = load_fn(args["pointcloud"], args.find("yz") == args.end());
    
    Geometry cloud = og_cloud; // make a copy of original order for masking purposes
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
    
    Renderer renderer(cloud, args.find("windowed") == args.end());
    Camera camera(
        centroid(cloud),
        glm::vec3(0.0f, 0.0f, 1.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        1.2f,
        renderer.aspect(),
        0.1f,
        200.0f,
        ground_level
    );

    HUD hud(renderer.window(), camera, masks, renderer);

    user_inputs inputs, prev_inputs;
    bool to_move = false;
    while (!glfwWindowShouldClose(renderer.window()))
    {
        prev_inputs = inputs;
        glfwPollEvents();
        inputs = user_inputs::fetch(renderer.window());
        
        if (to_move && glfwGetInputMode(renderer.window(), GLFW_CURSOR) == GLFW_CURSOR_DISABLED)
            execute_movement(camera, inputs, prev_inputs, ImGui::GetIO().Framerate);
        else
            to_move = true;
        
        if (DOWN(LMB, inputs, prev_inputs))
        {
            renderer.enable_cursor();
            to_move = false;
        }
        else if (DOWN(RMB, inputs, prev_inputs))
            renderer.disable_cursor();

        if (K_ESC(inputs))
            break;

        mask_updated = false;
        for (size_t i = 0; i < masks.size(); ++i)
        {
            if (masks[i] != masks_applied[i])
            {
                mask_updated = true;
                if (mask_invert && masks[i].active)
                {
                    std::fill(masks_applied.begin(), masks_applied.end(), false);
                    for (auto &mask : masks)
                        mask.active = false;
                    masks_applied[i] = masks[i].active = true;
                    break;
                }
                else masks_applied[i] = masks[i].active;
            }
        }
        if (mask_updated)
        {
            cloud = og_cloud;
            // apply the masks in the opposite order
            for (auto msk_it = masks.rbegin(); msk_it != masks.rend(); ++msk_it)
            {
                if (msk_it->active)
                {
                    if (mask_invert)
                        msk_it->apply_inv(cloud);
                    else
                        msk_it->apply(cloud);
                }
            }
            to_move = false;
        }

        hud.configure();

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (mask_updated)
            renderer.draw(cloud, camera);
        else
            renderer.draw(camera);
        hud.render();

        glfwSwapBuffers(renderer.window());
    }
    return 0;
}
