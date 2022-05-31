
#include "point_ops.hpp"
#include <fstream>
#include <algorithm>

Mask load_mask(const std::string &filename)
{
    std::ifstream ifs(filename);
    if (!ifs)
        throw std::runtime_error("load_mask: failed to open file");

    Mask mask;
    while (ifs.peek() != EOF)
    {
        int i;
        ifs >> i;
        mask.push_back(i);
    }
    return mask;
}

void apply_mask(PointCloud &points, const Mask &mask, const glm::vec3 &color)
{
    if (points.size() != mask.size())
        throw std::runtime_error("apply_mask: cannot apply a mask with " + 
            std::to_string(mask.size()) + " points to a cloud with " + 
            std::to_string(points.size()) + " points.");

    std::transform(points.begin(), points.end(), mask.begin(), points.begin(),
        [color](XYZRGBD &pt, bool mask_val) {
            if (mask_val)
            {
                pt.rgb = color;
            }
            return pt;
        });
}