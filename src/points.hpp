
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

/**
 * Update the distance of each point in the point cloud with respect to the
 * camera position.
 * 
 * @param points point cloud to update.
 * @param camera_pos camera position.
 */
void update(PointCloud &points, const glm::vec3 &camera_pos);

/**
 * Resort the point cloud based on the distance to the camera when the camera
 * moves.
 * 
 * @param points point cloud to sort.
 */
void resort(PointCloud &points);

/**
 * Update and resort functions but they are asynchronous. Hence they create a
 * new copy of the point cloud and are meant to be run in a separate thread.
 */
PointCloud update_async(const PointCloud &points, const glm::vec3 &camera_pos);
PointCloud resort_async(const PointCloud &points);
PointCloud update_resort_async(const PointCloud &points, const glm::vec3 &camera_pos);
