
#include "voxel.hpp"
#include "renderer.hpp"
#include "input.hpp"
#include <algorithm>
#include <iostream>
#include <vector>

template<typename T>
using vv = std::vector<T>;

int main()
{
    auto b = read_voxels("../tmp/voxel/1");
    auto [b_pv, b_pi] = voxel_geometry(0.02f, b.pt_voxels);
    auto [b_gv, b_gi] = voxel_geometry(0.32f, b.gt_voxels);
    auto b_pf = repeat8(b.pt_colors);
    auto b_gf = repeat8(colorize(b.gt_labels));
    
    // std::cout << *std::max_element((float*)&*b_pf.begin(), (float*)&*b_pf.end()) << "\n";
    // std::cout << *std::min_element((float*)&*b_pf.begin(), (float*)&*b_pf.end()) << std::endl;
    // return 1;
    VoxelRenderer renderer(b_pv, b_pi, b_pf, b_gv, b_gi, b_gf, false);
    Camera camera (
        glm::vec3(0.0f, 0.0f, 0.0f),
        glm::vec3(0.0f, 0.0f, -1.0f),
        glm::vec3(0.0f, 1.0f, 0.0f),
        1.2f,
        renderer.aspect(),
        0.1f,
        100.0f,
        0.0f
    );

    user_inputs inputs, prev_inputs;
    bool to_move = false;
    while (!glfwWindowShouldClose(renderer.window()))
    {
        prev_inputs = inputs;
        glfwPollEvents();
        inputs = user_inputs::fetch(renderer.window());
        if (to_move && glfwGetInputMode(renderer.window(), GLFW_CURSOR) == GLFW_CURSOR_DISABLED)
            execute_movement(camera, inputs, prev_inputs, 60.0f);
        else
            to_move = true;

        if (K_ESC(inputs))
            break;

        if (DOWN(LMB, inputs, prev_inputs))
        {
            renderer.enable_cursor();
            to_move = false;
        }
        else if (DOWN(RMB, inputs, prev_inputs))
            renderer.disable_cursor();
        
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.draw(camera);

        glfwSwapBuffers(renderer.window());
    }

    return 0;
}