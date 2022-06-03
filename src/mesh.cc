
#include "mesh.hpp"


namespace std
{
    // hash for eigen vec3d
    template<>
    struct hash<Eigen::Vector3d>
    {
        std::size_t operator()(const Eigen::Vector3d& v) const
        {
            return std::hash<double>()(v[0]) ^ std::hash<double>()(v[1]) ^ std::hash<double>()(v[2]);
        }
    };
}

o3d_PointCloud load_mesh(const std::string &filename, bool exchange_yz)
{    
    o3d_PointCloud pc;
    if (!open3d::io::ReadTriangleMesh(filename, pc))
        throw std::runtime_error("load: cannot load mesh from " + mesh);
    if (!pc.HasVertices())
        throw std::runtime_error("load: mesh has no vertices");
    if (!pc.HasTriangles())
        throw std::runtime_error("load: mesh has no triangles");
    
    if (exchange_yz)
    {
#pragma omp parallel for
        for (auto &v : pc.vertices_)
        {
            auto tmp = v.y();
            v.y() = v.z();
            v.z() = -tmp;
        }
#pragma omp parallel for
        for (auto &v : pc.vertex_normals_)
        {
            auto tmp = v.y();
            v.y() = v.z();
            v.z() = -tmp;
        }
    }

}