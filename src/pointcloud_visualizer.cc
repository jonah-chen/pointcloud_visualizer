
#include "renderer.hpp"
#include "points.hpp"
#include "point_ops.hpp"
#include "input.hpp"
#include "hud.hpp"
#include "parser.hpp"

int main(int argc, char** argv)
{
    auto args = parse_args(argc, argv);

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
    
    
    PointRenderer renderer(cloud, args.find("windowed") == args.end());
    Camera camera(
        centroid(cloud),            // position
        glm::vec3(0.0f, 0.0f, 1.0f),// forward
        glm::vec3(0.0f, 1.0f, 0.0f),// up
        1.2f,                       // fov
        renderer.aspect(),          // aspect
        0.1f,                       // near plane
        200.0f,                     // far plane
        ground_level                // ground plane
    );


    PointHUD hud(renderer.window(), camera, renderer, masks);
    user_inputs inputs, prev_inputs;
    bool to_move = false;

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
            
            to_move = false;
        }

        hud.configure();

        // Rendering
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
