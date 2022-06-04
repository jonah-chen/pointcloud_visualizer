
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <glm/glm.hpp>
#include <string>
#include <vector>

using pcl_PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
struct XYZRGB
{
    union
    {
        glm::vec3 xyz;
        struct
        {
            float x, y, z;
        };
    };
    union
    {
        glm::vec3 rgb;
        struct
        {
            float r, g, b;
        };
    };
    float d_sq;

    XYZRGB();
    XYZRGB(const XYZRGB &pt) = default;

    XYZRGB(const pcl::PointXYZRGB &pt, bool exchange_yz=false);
};

using PointCloud = std::vector<XYZRGB>;

/**
 * Load a point cloud from a .ply or .pcd file into a renderable format with
 * respect to a given camera position.
 * 
 * @param filename name of the file to load.
 * @param camera_pos camera position.
 * @param exchange_xz flag to specify if the x and z axes are exchanged.
 * @return PointCloud the loaded point cloud.
 */
PointCloud load(const std::string &filename, bool exchange_xz);
