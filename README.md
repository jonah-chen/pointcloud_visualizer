# Pointcloud Visualizer

I am sick of visualizing point clouds using the tools I can find like MATLAB or MeshLab. They all seem like CAD software and you move around the world like if they are CAD software. I have CAD software. So it's time to write my own. It won't be as complex or fully featured as the other ones, but I will be able to move/look around in the 3D scene as if I were in a 3D game like minecraft.

This repo will mainly include features tailored to visualizing results from 3D instance segmentation and object detection on point clouds. I will use OpenGL, ImGui and GLFW on linux. As this repo is just for my group, I will not make any efforts for allowing this to build with Windows.

### TODO:
1. Test the code with selecting instance masks
2. Render the tightest bounding boxes and other bounding boxes