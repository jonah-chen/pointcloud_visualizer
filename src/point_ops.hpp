
#pragma once

#include "points.hpp"

struct Mask
{
    std::vector<bool> mask;
    glm::vec3 color;
    std::string cls;
    bool active { false };

    inline bool operator==(bool other_active) const
    { return active == other_active; }
    inline bool operator!=(bool other_active) const
    { return active != other_active; }

    operator bool() const { return active; }

    Mask(const std::string &filename, const glm::vec3 &color, const std::string &cls);
    void apply(PointCloud &points);
};

struct BBox
{
    glm::vec3 bl;
    glm::vec3 tr;
    BBox();
    BBox(const glm::vec3 &_x, const glm::vec3 &_y, bool center_size_fmt=true);
};

std::vector<Mask> load_masks(const std::string &filename);

BBox tightest_bbox(const PointCloud &points, const Mask &mask);
