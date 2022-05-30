
#include "points.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

PointCloud::Ptr load(const std::string &filename)
{
    PointCloud::Ptr cloud = std::make_shared<PointCloud>();
    if (filename.substr(filename.find_last_of(".") + 1) == "ply")
        pcl::io::loadPLYFile(filename, *cloud);
    else if (filename.substr(filename.find_last_of(".") + 1) == "pcd")
        pcl::io::loadPCDFile(filename, *cloud);
    else
        throw std::runtime_error("load: unknown file extension");
    return cloud;
}


