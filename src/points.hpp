
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
PointCloud::Ptr load(const std::string &filename);

void initRenderer();
