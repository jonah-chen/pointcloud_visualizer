
#include "points.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <glm/gtx/norm.hpp>
#include <cstring>
#include <limits>

std::vector<XYZRGBD> load(const std::string &filename, 
                          const glm::vec3 &camera_pos,
                          bool exchange_xz)
{
    PointCloud::Ptr cloud = std::make_shared<PointCloud>();
    if (filename.substr(filename.find_last_of(".") + 1) == "ply")
        pcl::io::loadPLYFile(filename, *cloud);
    else if (filename.substr(filename.find_last_of(".") + 1) == "pcd")
        pcl::io::loadPCDFile(filename, *cloud);
    else
        throw std::runtime_error("load: unknown file extension");
    
    std::vector<XYZRGBD> points;
    points.reserve(cloud->size());
    for (const auto &pt : *cloud)
        points.emplace_back(pt, camera_pos, exchange_xz);
    std::sort(points.begin(), points.end(), std::greater<XYZRGBD>());
    return points;
}

XYZRGBD::XYZRGBD()
{
    memset(this, 0, sizeof(XYZRGBD));
}

XYZRGBD::XYZRGBD(const pcl::PointXYZRGB &pt)
{
    xyz = glm::vec3(pt.x, pt.y, pt.z);
    r = pt.r / 255.0f;
    g = pt.g / 255.0f;
    b = pt.b / 255.0f;
    d_sq = std::numeric_limits<float>::infinity();
}

XYZRGBD::XYZRGBD(const pcl::PointXYZRGB &pt, float d)
{
    xyz = glm::vec3(pt.x, pt.y, pt.z);
    r = pt.r / 255.0f;
    g = pt.g / 255.0f;
    b = pt.b / 255.0f;
    d_sq = d * d;
}

XYZRGBD::XYZRGBD(const pcl::PointXYZRGB &pt, const glm::vec3 &camera_pos, bool exchange_yz)
{
    if (exchange_yz)
        xyz = glm::vec3(pt.x, pt.z, -pt.y);
    else
        xyz = glm::vec3(pt.x, pt.y, pt.z);

    r = pt.r / 255.0f;
    g = pt.g / 255.0f;
    b = pt.b / 255.0f;
    d_sq = glm::distance2(xyz, camera_pos);
}

void XYZRGBD::update(const glm::vec3 &camera_pos)
{
    d_sq = glm::distance2(xyz, camera_pos);
}

void update(std::vector<XYZRGBD> &points, const glm::vec3 &camera_pos)
{
    for (auto &pt : points)
        pt.update(camera_pos);
    std::sort(points.begin(), points.end(), std::greater<XYZRGBD>());
}
