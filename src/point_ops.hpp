
#pragma once

#include "points.hpp"

Mask load_mask(const std::string &filename);

void apply_mask(PointCloud &points, const Mask &mask, const glm::vec3 &color);