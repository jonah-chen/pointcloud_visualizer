
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <glm/glm.hpp>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;

struct XYZRGBD
{
    union
    {
        glm::vec3 xyz;
        struct
        {
            float x, y, z;
        };
    };
    float r, g, b;
    float d_sq;

    XYZRGBD();
    XYZRGBD(const pcl::PointXYZRGB &pt);
    XYZRGBD(const pcl::PointXYZRGB &pt, float d);
    XYZRGBD(const pcl::PointXYZRGB &pt, const glm::vec3 &camera_pos, bool exchange_yz=false);

    /**
     * Update the distance based on the new camera position.
     * 
     * @param camera_pos camera position.
     */
    void update(const glm::vec3 &camera_pos);

    inline bool operator<(const XYZRGBD &rhs) const { return d_sq < rhs.d_sq; }
    inline bool operator>(const XYZRGBD &rhs) const { return d_sq > rhs.d_sq; }
};

std::vector<XYZRGBD> load(const std::string &filename, const glm::vec3 &camera_pos, bool exchange_xz);

void update(std::vector<XYZRGBD> &points, const glm::vec3 &camera_pos);
