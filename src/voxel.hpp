
#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <filesystem>
#include <optional>

using VoxelGeo = std::pair<std::vector<glm::vec3>, std::vector<unsigned int>>;

VoxelGeo voxel_geometry(float voxel_size, const std::vector<glm::ivec3> &voxels, float rot_angle = 0.0f);

template<typename T>
std::vector<T> repeat8(const std::vector<T> &properties)
{
    std::vector<T> result(8 * properties.size());
#pragma omp parallel for
    for (std::size_t i = 0; i < properties.size(); ++i)
        for (std::size_t j = 0; j < 8; ++j)
            result[i * 8 + j] = properties[i];
    return result;
}

std::vector<glm::vec3> colorize(const std::vector<uint16_t> &labels, std::optional<std::vector<glm::vec3>> colors = std::nullopt);

struct SceneInstance
{
    std::vector<glm::ivec3> gt_voxels, pt_voxels;
    std::vector<glm::vec3> pt_colors;
    std::vector<uint16_t> gt_labels;
};

SceneInstance read_voxels(std::filesystem::path path);
