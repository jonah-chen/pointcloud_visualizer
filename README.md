# Pointcloud Visualizer

I am sick of visualizing point clouds using the tools I can find like MATLAB or MeshLab. They all seem like CAD software and you move around the world like if they are CAD software. I have CAD software. So it's time to write my own. It won't be as complex or fully featured as the other ones, but I will be able to move/look around in the 3D scene as if I were in a 3D game like minecraft.

This repo will mainly include features tailored to visualizing results from 3D instance segmentation and object detection on point clouds. I will use OpenGL, ImGui and GLFW on linux. As this repo is just for my group, I will not make any efforts for allowing this to build with Windows.

## Usage

Make sure you have the core dependencies installed (GLFW, Eigen, etc). Then install open3d by executing `bash install_deps.sh`.

- If there is a scene in the S3DIS dataset, convert it to ply with `python utils/txt2ply.py <PATH_TO_TXT>`
- Or, if you want to change the scene to semantic labels stored in numpy array with shape (num_pts,), then execute `python utils/apply_semantic.py <PATH_TO_TXT> <PATH_TO_NPY>`
- If you want to visualize some center shifts stored in a numpy array with shape (num_pts,3,), then execute `python utils/apply_offsets.py <PATH_TO_TXT> <PATH_TO_NPY>`
- Please prepare your masks and a `.msk` file with the following format. This can be done automatically if your output is similar to SoftGroup by executing `python utils/softgroup2msk.py <PATH_TO_SOFTGROUP_OUTPUT>`.
```
 <mask_filepath (.txt)> <mask_color (hex code)> <mask_class>
 <mask_filepath (.txt)> <mask_color (hex code)> <mask_class>
 ...
```

To visualize, you may build the project with
```
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
```

- If you want to generate meshes from point cloud using BPA or Poission methods, execute `./BPA <PATH_TO_INPUT_PLY> <PATH_TO_OUTPUT_PLY>` or `./Poission <PATH_TO_INPUT_PLY> <PATH_TO_OUTPUT_PLY> <depth (optional)>`.

- To visualize point cloud only, execute `./Pointcloud_Visualizer <PATH_TO_PLY>`.
- To visualize a mesh (generated or native), execute `./Meshed_Visualizer <PATH_TO_PLY>`.
- For both mesh and point cloud, you can specify the following flags
```
--masks   : specify a file path to a .msk file to load the instance masks for the scene. These will be able to be toggled on/off.
--yz      : swap the y and z coordinates (and invert the z coordinate) of the input file.
--windowed: start the program in windowed mode.
--invert  : instead of coloring in the mask, gray out all points that are not in the mask.
```


### TODO:
~~1. Test the code with selecting instance masks~~
2. Render the tightest bounding boxes and other bounding boxes
~~3. Learn to do [normal estimation](https://pcl-docs.readthedocs.io/en/latest/pcl/doc/tutorials/content/normal_estimation.html) on 3D scans that does not come with normals~~
~~4. Explore different methods of mesh reconstruction to be able to render meshes from pointcloud. Then, attempt to create instance masks with meshed objects, starting with [this](https://towardsdatascience.com/5-step-guide-to-generate-3d-meshes-from-point-clouds-with-python-36bad397d8ba).~~
~~5. Learn to use a Z-Buffer to be able to render the triangle meshes without sorting.~~
