
#include "open3d/Open3D.h"
#include <algorithm>
#include <numeric>

int main(int argc, char *argv[]) 
{
    if (argc < 3)
    {
        std::cout << "Usage: " << argv[0] << " <input_file> <output_file>" << std::endl;
        return -1;
    }
    open3d::geometry::PointCloud pc;
    
    if (!open3d::io::ReadPointCloud(argv[1], pc))
    {
        std::cerr << "Failed to read point cloud from " << argv[1] << std::endl;
        return EXIT_FAILURE;
    }

    auto distances = pc.ComputeNearestNeighborDistance();
    double avg_distance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
    double rho = 1.25 * avg_distance / 2.0;
    std::vector<double> radii { 0.1 * rho, 0.5 * rho, rho, 2.0 * rho, 10.0 * rho };

    auto bpa_mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(pc, radii);
    
    if (!open3d::io::WriteTriangleMesh(argv[2], *bpa_mesh))
    {
        std::cerr << "Failed to write mesh to " << argv[2] << std::endl;
        return EXIT_FAILURE;
    }
    return 0;
}