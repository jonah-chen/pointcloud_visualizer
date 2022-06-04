
#include "points.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <glm/gtx/norm.hpp>
#include <cstring>
#include <limits>

PointCloud load(const std::string &filename, bool exchange_xz)
{
    pcl_PointCloud::Ptr cloud ( new pcl_PointCloud );
    if (filename.substr(filename.find_last_of(".") + 1) == "ply")
        pcl::io::loadPLYFile(filename, *cloud);
    else if (filename.substr(filename.find_last_of(".") + 1) == "pcd")
        pcl::io::loadPCDFile(filename, *cloud);
    else
        throw std::runtime_error("load: unknown file extension");

    if (!cloud || cloud->points.empty())
        throw std::runtime_error("load: cannot load point cloud");

    PointCloud points;
    points.reserve(cloud->size());
    for (const auto &pt : *cloud)
        points.emplace_back(pt, exchange_xz);
    return points;
}

XYZRGB::XYZRGB()
{
    memset(this, 0, sizeof(XYZRGB));
}

XYZRGB::XYZRGB(const pcl::PointXYZRGB &pt, bool exchange_yz)
{
    xyz = exchange_yz ? glm::vec3(pt.x, pt.z, -pt.y) : glm::vec3(pt.x, pt.y, pt.z);
    rgb = glm::vec3(pt.r, pt.g, pt.b) / 255.0f;
}