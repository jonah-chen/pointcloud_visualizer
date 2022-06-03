
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

m_PointCloud load_mesh(const std::string &filename, bool exchange_yz)
{    
    o3d_PointCloud pc;
    if (!open3d::io::ReadTriangleMesh(filename, pc))
        throw std::runtime_error("load: cannot load mesh from " + filename);
    if (!pc.HasVertices())
        throw std::runtime_error("load: mesh has no vertices");
    if (!pc.HasTriangles())
        throw std::runtime_error("load: mesh has no triangles");
    if (!pc.HasVertexNormals())
        throw std::runtime_error("load: mesh has no vertex normals");
    if (!pc.HasVertexColors())
        throw std::runtime_error("load: mesh has no vertex colors");
    if (pc.vertices_.size() != pc.vertex_colors_.size())
        throw std::runtime_error("load: mesh has different number of vertices and vertex colors");
    if (pc.vertices_.size() != pc.vertex_normals_.size())
        throw std::runtime_error("load: mesh has different number of vertices and vertex normals");

    m_PointCloud vertices(pc.vertices_.size());

    if (exchange_yz)
    {
#pragma omp parallel for
        for (size_t i = 0; i < pc.vertices_.size(); ++i)
        {
            auto &p = pc.vertices_[i];
            auto &n = pc.vertex_normals_[i];
            auto &c = pc.vertex_colors_[i];
            vertices.v[i].p = {p.x(), p.z(), -p.y()};
            vertices.v[i].n = {n.x(), n.z(), -n.y()};
            vertices.v[i].c = {c.x(), c.y(), c.z()};
        }
    }
    else
    {   
#pragma omp parallel for
        for (size_t i = 0; i < pc.vertices_.size(); ++i)
        {
            auto &p = pc.vertices_[i];
            auto &n = pc.vertex_normals_[i];
            auto &c = pc.vertex_colors_[i];
            vertices.v[i].p = {p.x(), p.y(), p.z()}; 
            vertices.v[i].n = {n.x(), n.y(), n.z()};
            vertices.v[i].c = {c.x(), c.y(), c.z()};
        }
        return vertices;
    }
}


