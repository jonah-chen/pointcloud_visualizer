#!/bin/bash
if nvcc
then
    echo "Downloading Open3D with CUDA support"
    wget https://github.com/isl-org/Open3D/releases/download/v0.15.1/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.15.1.tar.xz
    tar -xf open3d-devel-linux-x86_64-cxx11-abi-cuda-0.15.1.tar.xz
    rm open3d-devel-linux-x86_64-cxx11-abi-cuda-0.15.1.tar.xz
    mv open3d-devel-linux-x86_64-cxx11-abi-cuda-0.15.1 open3d
else
    echo "CUDA is not found. Downloading Open3D without CUDA support"
    wget https://github.com/isl-org/Open3D/releases/download/v0.15.1/open3d-devel-linux-x86_64-cxx11-abi-0.15.1.tar.xz 
    tar -xf open3d-devel-linux-x86_64-cxx11-abi-0.15.1.tar.xz
    rm open3d-devel-linux-x86_64-cxx11-abi-0.15.1.tar.xz
    mv open3d-devel-linux-x86_64-cxx11-abi-0.15.1 open3d
fi

if mv ./open3d/include/open3d ./include/open3d
then
    echo "Moving Open3D headers to ./include/open3d"
fi

if mv ./open3d/lib/libOpen3D.so ./lib/libOpen3D.so
then
    echo "Moving Open3D library to ./lib/libOpen3D.so"
fi

rm -rf ./open3d
