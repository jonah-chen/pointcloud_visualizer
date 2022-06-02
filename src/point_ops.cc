
#include "point_ops.hpp"
#include <fstream>
#include <algorithm>
#include <limits>

BBox::BBox()
{
    memset(this, 0, sizeof(BBox));
}

BBox::BBox(const glm::vec3 &_x, const glm::vec3 &_y, bool center_size_fmt)
{
    if (center_size_fmt)
    {
        bl = _x - _y / 2.0f;
        tr = _x + _y / 2.0f;
    }
    else
    {
        bl = _x;
        tr = _y;
    }
}

Mask::Mask(const std::string &filename, const glm::vec3 &color, const std::string &cls)
    : color(color), cls(cls)
{
    std::ifstream ifs(filename);
    if (!ifs)
        throw std::runtime_error("load_mask: failed to open file " + filename);

    while (ifs.good())
    {
        int i;
        ifs >> i;
        if (ifs.eof())
            break;
        mask.push_back(i);
    }
}

void Mask::apply(PointCloud &points)
{
    if (points.size() != mask.size())
        throw std::runtime_error("apply_mask: cannot apply a mask with " + 
            std::to_string(mask.size()) + " points to a cloud with " + 
            std::to_string(points.size()) + " points.");
    
#pragma omp parallel for
    for (size_t i = 0; i < points.size(); ++i)
        if (mask[i])
            points[i].rgb = color;
}

std::vector<Mask> load_masks(const std::string &filename)
{
    std::ifstream ifs(filename);
    if (!ifs)
        throw std::runtime_error("load_masks: failed to open file " + filename);

    std::vector<Mask> masks;
    std::string fileroot;

    auto _i = filename.find_last_of('/');
    if (_i != std::string::npos)
        fileroot = filename.substr(0, _i + 1);

    while (ifs.peek() != EOF)
    {
        std::string fn;
        std::string cls;
        std::string hex;
        ifs >> fn >> hex >> cls;
    
        uint8_t r = std::stoi(hex.substr(0, 2), nullptr, 16);
        uint8_t g = std::stoi(hex.substr(2, 2), nullptr, 16);
        uint8_t b = std::stoi(hex.substr(4, 2), nullptr, 16);
        glm::vec3 color(r / 255.0f, g / 255.0f, b / 255.0f);

        masks.emplace_back(fileroot + fn, color, cls);
    }
    return masks;
}

BBox tightest_bbox(const PointCloud &points, const Mask &mask)
{
    BBox bbox;
    bbox.bl = glm::vec3(std::numeric_limits<float>::max(), 
                        std::numeric_limits<float>::max(),
                        std::numeric_limits<float>::max());
    bbox.tr = glm::vec3(std::numeric_limits<float>::min(),
                        std::numeric_limits<float>::min(),
                        std::numeric_limits<float>::min());

    for (size_t i = 0; i < points.size(); ++i)
    {
        if (mask.mask[i])
        {
            bbox.bl.x = std::min(bbox.bl.x, points[i].xyz.x);
            bbox.bl.y = std::min(bbox.bl.y, points[i].xyz.y);
            bbox.bl.z = std::min(bbox.bl.z, points[i].xyz.z);
            bbox.tr.x = std::max(bbox.tr.x, points[i].xyz.x);
            bbox.tr.y = std::max(bbox.tr.y, points[i].xyz.y);
            bbox.tr.z = std::max(bbox.tr.z, points[i].xyz.z);
        }
    }

    if (bbox.bl.x == std::numeric_limits<float>::max())
        throw std::runtime_error("tightest_bbox: no points in mask");

    return bbox;
}

glm::vec3 centroid(const PointCloud &points)
{
    glm::dvec3 c;
    for (auto &p : points)
        c += p.xyz;
    return c / static_cast<double>(points.size());
}


glm::vec3 centroid(const PointCloud &points, const Mask &mask)
{
    glm::dvec3 c;
    int count = 0;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (mask.mask[i])
        {
            c += points[i].xyz;
            ++count;
        }
    }
    return c / static_cast<double>(count);
}
