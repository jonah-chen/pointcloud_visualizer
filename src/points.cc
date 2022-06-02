
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

XYZRGBD::XYZRGBD()
{
    memset(this, 0, sizeof(XYZRGBD));
}

XYZRGBD::XYZRGBD(const pcl::PointXYZRGB &pt, bool exchange_yz, float d)
{
    xyz = exchange_yz ? glm::vec3(pt.x, pt.z, -pt.y) : glm::vec3(pt.x, pt.y, pt.z);
    rgb = glm::vec3(pt.r, pt.g, pt.b) / 255.0f;
    d_sq = d * d;
}

XYZRGBD::XYZRGBD(const pcl::PointXYZRGB &pt, const glm::vec3 &camera_pos, bool exchange_yz)
{
    xyz = exchange_yz ? glm::vec3(pt.x, pt.z, -pt.y) : glm::vec3(pt.x, pt.y, pt.z);
    rgb = glm::vec3(pt.r, pt.g, pt.b) / 255.0f;
    d_sq = glm::distance2(xyz, camera_pos);
}

void XYZRGBD::update(const glm::vec3 &camera_pos)
{
    d_sq = glm::distance2(xyz, camera_pos);
}

void update(PointCloud &points, const glm::vec3 &camera_pos)
{
#pragma omp parallel for
    for (size_t i = 0; i < points.size(); ++i)
        points[i].update(camera_pos);
}

void resort(PointCloud &points)
{
    std::sort(points.begin(), points.end(), std::greater<XYZRGBD>());
}

PointCloud update_async(const PointCloud &points, const glm::vec3 &camera_pos)
{
    PointCloud new_points(points);
#pragma omp parallel for
    for (auto &pt : new_points)
        pt.update(camera_pos);
    return new_points;
}

PointCloud resort_async(const PointCloud &points)
{
    PointCloud new_points(points);
    std::sort(new_points.begin(), new_points.end(), std::greater<XYZRGBD>());
    return new_points;
}

PointCloud update_resort_async(const PointCloud &points, const glm::vec3 &camera_pos)
{
    PointCloud new_points(points);
#pragma omp parallel for
    for (auto &pt : new_points)
        pt.update(camera_pos);

    std::sort(new_points.begin(), new_points.end(), std::greater<XYZRGBD>());
    return new_points;
}
