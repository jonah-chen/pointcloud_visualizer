
#include "open3d/Open3D.h"
#include <algorithm>
#include <numeric>

int main(int argc, char *argv[]) 
{
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <input_file> <output_file>" << std::endl;
        return EXIT_SUCCESS;
    }
    open3d::geometry::PointCloud pc;
    
    if (!open3d::io::ReadPointCloud(argv[1], pc))
    {
        std::cerr << "Failed to read point cloud from " << argv[1] << std::endl;
        return EXIT_FAILURE;
    }

    auto poission_output = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(pc, 12);
    auto poission_mesh = std::get<0>(poission_output);

    if (!open3d::io::WriteTriangleMesh(argv[2], *poission_mesh))
    {
        std::cerr << "Failed to write mesh to " << argv[2] << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}