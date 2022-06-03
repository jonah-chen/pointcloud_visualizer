
#pragma once

#include "points.hpp"
#include "mesh.hpp"

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

    Mask(Mask &&other);

    /**
     * Apply the mask to the given point cloud.
     * 
     * @param points the point cloud to apply the mask to.
     * @note the point cloud must have the same number of points as the mask.
     */
    void apply(PointCloud &points);

    void apply(o3d_PointCloud &points);
};

struct BBox
{
    glm::vec3 bl;
    glm::vec3 tr;
    BBox();
    BBox(const glm::vec3 &_x, const glm::vec3 &_y, bool center_size_fmt=true);
};

/**
 * Load a set of masks from a masks file of the format:
 *      <mask_filepath (.txt)> <mask_color (hex code)> <mask_class>
 *      ...
 * 
 * @param filename name of the masks file.
 * @return std::vector<Mask> the loaded masks.
 */
std::vector<Mask> load_masks(const std::string &filename);

/**
 * Extract the tight bounding box of a given mask on a point cloud.
 * 
 * @param points point cloud the mask is on.
 * @param mask mask to extract the bounding box from.
 * @return BBox the tight bounding box of the mask.
 */
BBox tightest_bbox(const PointCloud &points, const Mask &mask);

/**
 * Find the center of a given point cloud.
 * 
 * @param points point cloud to find the center of.
 * @return glm::vec3 the center of the point cloud.
 */
glm::vec3 centroid(const PointCloud &points);

/**
 * Find the center of a given object in the scene.
 * 
 * @param points point cloud of the scene.
 * @param mask mask of the object to find the center of.
 * @return glm::vec3 the center of the object.
 */
glm::vec3 centroid(const PointCloud &points, const Mask &mask);
