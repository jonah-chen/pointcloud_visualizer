
#include "voxel.hpp"
#include "npy/npy.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <stdexcept>
#include <array>
#include <filesystem>
#include <iostream>
#include <cassert>
#include <cstdint>

VoxelGeo voxel_geometry(float voxel_size, const std::vector<glm::ivec3> &voxels, float rot_angle)
{
    // generate the xy rotation matrix
    glm::mat4 rot_mat = glm::rotate(glm::mat4(1.0f), rot_angle, glm::vec3(0.0f, 0.0f, 1.0f));
    
    // get number of voxels
    const std::size_t num_voxels = voxels.size();
    if (num_voxels == 0)
        throw std::runtime_error("no voxels");

    // generate cube indices for each voxel
    std::vector<unsigned int> indices(num_voxels * 36);

    // index offset for cube
    constexpr std::array<unsigned int, 36> index_offset {{
        0, 1, 2, 0, 2, 3,
        4, 5, 6, 4, 6, 7,
        0, 1, 5, 0, 5, 4,
        2, 3, 7, 2, 7, 6,
        1, 2, 6, 1, 6, 5,
        0, 3, 7, 0, 7, 4
    }};
#pragma omp parallel for
    for (std::size_t i = 0; i < num_voxels; ++i)
        for (std::size_t j = 0; j < 36; ++j)
            indices[i * 36 + j] = index_offset[j] + i * 8;

    // generate cube vertices for each voxel
    std::vector<glm::vec3> vertices(num_voxels * 8);
#pragma omp parallel for
    for (std::size_t i = 0; i < num_voxels; ++i)
    {
        // get voxel position
        const glm::ivec3 &voxel_pos = voxels[i];
        const glm::vec3 voxel_pos_vec = glm::vec3(voxel_pos);
        
        // get voxel rotation
        const glm::mat4 voxel_rot = rot_mat * glm::mat4(voxel_size);
        
        // get voxel vertices
        const glm::vec3 voxel_vertices[8] = {
            glm::vec3(0.0f, 0.0f, 0.0f) + voxel_pos_vec,
            glm::vec3(1.0f, 0.0f, 0.0f) + voxel_pos_vec,
            glm::vec3(1.0f, 1.0f, 0.0f) + voxel_pos_vec,
            glm::vec3(0.0f, 1.0f, 0.0f) + voxel_pos_vec,
            glm::vec3(0.0f, 0.0f, 1.0f) + voxel_pos_vec,
            glm::vec3(1.0f, 0.0f, 1.0f) + voxel_pos_vec,
            glm::vec3(1.0f, 1.0f, 1.0f) + voxel_pos_vec,
            glm::vec3(0.0f, 1.0f, 1.0f) + voxel_pos_vec
        };
        
        // get voxel vertices transformed by rotation
        for (std::size_t j = 0; j < 8; ++j)
            vertices[i * 8 + j] = voxel_rot * glm::vec4(voxel_vertices[j], 1.0f);
    }

    // return voxel geometry
    return std::make_pair(vertices, indices);
}

std::vector<glm::vec3> colorize(const std::vector<uint16_t> &labels, cmap::fn col)
{
    // get the max label
    const uint16_t max_label = *std::max_element(labels.begin(), labels.end());
    std::vector<glm::vec3> result;
    result.reserve(labels.size());
    for (auto &&label : labels)
        result.emplace_back(col(label / (float)max_label));
    return result;
}


namespace {
template<typename T>
std::pair<std::vector<T>, uint64_t> read(std::filesystem::path path, uint64_t expected_feats)
{
    std::vector<uint64_t> shape;
    bool fortran_order;
    std::vector<T> data;

    npy::LoadArrayFromNumpy(path, shape, fortran_order, data);
    if (fortran_order)
        throw std::runtime_error("fortran order not supported");
    if (shape.size() != 2)
        throw std::runtime_error("it must be a 2D array");
    if (shape[1] != expected_feats)
        throw std::runtime_error("it must have " + std::to_string(expected_feats) + " features");
    return std::make_pair(data, shape[0]);
}

std::vector<glm::ivec3> parse_voxel_idx(const std::vector<int64_t> &np_arr, uint64_t size)
{
    std::vector<glm::ivec3> voxel_idx;
    voxel_idx.reserve(size);
    for (std::size_t i = 0; i < size; ++i)
    {
        assert(np_arr[i * 4] == 0);
        auto x = np_arr[i * 4 + 1];
        auto y = np_arr[i * 4 + 2];
        auto z = np_arr[i * 4 + 3];
        voxel_idx.emplace_back(x, z, -y);
    }
    return voxel_idx;
}

}


SceneInstance read_voxels(std::filesystem::path path)
{
    SceneInstance ret;
    // add path to filename
    auto Pgt_coords = path/"gt_coords.npy";
    auto Pgt_feats = path/"gt_feats.npy";
    auto Ppt_coords = path/"pt_coords.npy";
    auto Ppt_feats = path/"pt_feats.npy";

    auto [gt_coords, gt_coords_len] = read<int64_t>(Pgt_coords, 4); // 0, x, y, z
    auto [gt_feats, gt_feats_len] = read<float>(Pgt_feats, 1); // class
    auto [pt_coords, pt_coords_len] = read<int64_t>(Ppt_coords, 4); // 0, x, y, z
    auto [pt_feats, pt_feats_len] = read<float>(Ppt_feats, 3); // r, g, b

    ret.gt_voxels = parse_voxel_idx(gt_coords, gt_coords_len);
    ret.pt_voxels = parse_voxel_idx(pt_coords, pt_coords_len);

    ret.pt_colors.reserve(pt_coords_len);
    for (std::size_t i = 0; i < pt_coords_len; ++i)
    {
        float r[3];
        for (std::size_t j = 0; j < 3; ++j)
        {
            if (pt_feats[i * 3 + j] < -1.0f)
                r[j] = 0.0f;
            else if (pt_feats[i * 3 + j] > 1.0f)
                r[j] = 1.0f;
            else
                r[j] = (pt_feats[i * 3 + j] + 1.0f) / 2.0f;
        }
        ret.pt_colors.emplace_back(r[0], r[1], r[2]);
    }
    ret.gt_labels.reserve(gt_coords_len);
    for (std::size_t i = 0; i < gt_coords_len; ++i)
        ret.gt_labels.emplace_back(gt_feats[i]);
    
    return ret;
}
