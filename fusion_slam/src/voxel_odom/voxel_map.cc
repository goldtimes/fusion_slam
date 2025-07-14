#include "voxel_odom/voxel_map.hh"
#include <sys/types.h>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "common/logger.hh"

namespace slam {

void OctoTree::init_octo_tree() {
}

// 构建地图
void buildVoxelMap(const std::vector<PointWithCov>& input_points, const float voxel_size, const int max_layer,
                   const std::vector<int>& layer_point_size, const int max_points_size, const int max_cov_points_size,
                   const float plane_thresh, std::unordered_map<VOXEL_KEY, OctoTree*>& feat_map) {
    uint point_size = input_points.size();
    // 遍历所有的点，计算体素坐标
    for (uint i = 0; i < point_size; ++i) {
        const PointWithCov p_v = input_points[i];
        float loc_xyz[3];
        // 计算索引
        for (int j = 0; j < 3; ++j) {
            loc_xyz[j] = p_v.point[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_KEY position(static_cast<int64_t>(loc_xyz[0]), static_cast<int64_t>(loc_xyz[1]),
                           static_cast<int64_t>(loc_xyz[2]));
        auto iter = feat_map.find(position);
        // 不存在该体素
        if (iter == feat_map.end()) {
            feat_map[position]->temp_points_.push_back(p_v);
            feat_map[position]->new_points_num_++;
        } else {
            // 创建octotree
            OctoTree* octo_tree =
                new OctoTree(max_layer, 0, layer_point_size, max_points_size, max_cov_points_size, plane_thresh);
            // 设置八叉树相关的变量
            feat_map[position] = octo_tree;
            feat_map[position]->quater_length_ = voxel_size / 4;
            // 体素的中心
            feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            feat_map[position]->temp_points_.push_back(p_v);
            feat_map[position]->new_points_num_++;
            feat_map[position]->layer_point_size_ = layer_point_size;
        }
    }
    // 遍历所有的feat_map
    for (auto it = feat_map.begin(); it != feat_map.end(); ++it) {
        // 尝试初始化平面
        it->second->init_octo_tree();
    }
}

void updateVoxelMapOMP(const std::vector<PointWithCov>& input_points, const float voxel_size, const int max_layer,
                       const std::vector<int>& layer_point_size, const int max_points_size,
                       const int max_cov_points_size, const float plane_thresh,
                       std::unordered_map<VOXEL_KEY, OctoTree*>& feat_map) {
    std::unordered_map<VOXEL_KEY, std::vector<PointWithCov>> position_updates;
    int insert_num = 0, update_num = 0;
    uint point_size = input_points.size();
    auto start_time = std::chrono::high_resolution_clock::now();
    for (uint i = 0; i < point_size; ++i) {
        const PointWithCov p_v = input_points[i];
        float loc_xyz[3];
        // 计算索引
        for (int j = 0; j < 3; ++j) {
            loc_xyz[j] = p_v.point[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_KEY position(static_cast<int64_t>(loc_xyz[0]), static_cast<int64_t>(loc_xyz[1]),
                           static_cast<int64_t>(loc_xyz[2]));
        auto iter = feat_map.find(position);
        if (iter == feat_map.end()) {
            // 插入新的点，插入的点总是少的
            insert_num++;
            // 创建树节点
            OctoTree* octo_tree =
                new OctoTree(max_layer, 0, layer_point_size, max_points_size, max_cov_points_size, plane_thresh);
            feat_map[position] = octo_tree;
            feat_map[position]->quater_length_ = voxel_size / 4;
            // 体素的中心
            feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            feat_map[position]->UpdateOctoTree(p_v);
        } else {
            update_num++;
            position_updates[position].push_back(p_v);
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>((end_time - start_time)).count();
    LOG_INFO("insert voxel use time:{}", duration * 1e-3);
    start_time = std::chrono::high_resolution_clock::now();
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for default(none) shared(position_updates, feat_map);
#endif
    //  先遍历bucket 理想情况下bucket一般只有一个元素 这样还是相当于完全并行的遍历position_index_map
    for (size_t b = 0; b < position_updates.bucket_count(); b++) {
        for (auto bi = position_updates.begin(b); bi != position_updates.end(b); ++bi) {
            VOXEL_KEY position = bi->first;
            for (const auto& pv : bi->second) {
                feat_map[position]->UpdateOctoTree(pv);
            }
        }
    }
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>((end_time - start_time)).count();
    LOG_INFO("update voxel use time:{}", duration * 1e-3);
    LOG_INFO("update voxel size:{}, insert voxel_size:{}", update_num, insert_num);
}
}  // namespace slam