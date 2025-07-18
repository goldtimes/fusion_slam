#pragma once

#include <Eigen/Eigen>
#include <cstdint>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>
#include "common/common_lib.hh"
#define HASH_P 116101
#define MAX_N 10000000000

namespace slam {

struct VOXEL_KEY {
    int64_t x, y, z;
    VOXEL_KEY(int64_t vx, int64_t vy, int64_t vz) : x(vx), y(vy), z(vz) {
    }
    bool operator=(const VOXEL_KEY &other) const {
        return (x == other.x) && (y == other.y) && (z == other.z);
    }

    struct Hasher {
        int64_t operator()(const VOXEL_KEY &k) const {
            return ((((k.z) * HASH_P) % MAX_N + (k.y)) * HASH_P) % MAX_N + (k.x);
        }
    };
};

struct PointWithCov {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D point;  // 世界坐标系？
    M3D cov;
};
// 平面定义
struct Plane {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D normal;
    V3D center;
    V3D x_normal;
    V3D y_normal;
    M3D covariance;
    V3D eigens;
    Mat6d plane_cov;
    bool is_valid;
    int points_size;
};

struct ResidualData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d plane_center;
    Eigen::Vector3d plane_norm;
    Eigen::Matrix<double, 6, 6> plane_cov;
    // 这个是？
    Eigen::Matrix3d pcov;
    Eigen::Matrix3d cov;
    Eigen::Vector3d point_lidar;
    Eigen::Vector3d point_world;
    bool is_valid = false;
    // 是否找周边的体素
    bool from_near = false;
    int current_layer = 0;
    double sigma_num = 3.0;
    double residual = 0.0;
};

// 定义octotree
class OctoTree {
   public:
    OctoTree(int _max_layer, int _layer, std::vector<int> _update_size_threshes, int _max_point_thresh,
             double _plane_thresh);

    void insert(const std::vector<PointWithCov> &input_points);

    void initialize_tree();

    void build_plane(const std::vector<PointWithCov> &points);

    void split_tree();
    int subIndex(const PointWithCov &pv, int *xyz);

   public:
    int max_layer_;                                  // 最大的层数
    std::vector<int> max_layer_points_;              // 每层开始估计平面的阈值
    std::vector<std::shared_ptr<OctoTree>> leaves_;  // 子节点
    int current_layer_;                              // 当前层数
    bool is_initialized_;                            // 节点是否被初始化
    Plane plane_;                                    // 平面
    V3D voxel_center_;                               // 体素中心
    bool is_leave_;                                  // 叶子结点
    bool update_enable_;                             // 是否还需要更新
    int max_point_thresh_;                           // 不用在更新平面的阈值
    std::vector<PointWithCov> tmp_points_;           // 体素中的点
    int update_size_thresh_;                         // 更新的阈值
    double plane_thresh_;                            //平面阈值
    int all_point_num;                               // 体素中的全部点
    int new_point_num;                               // 新的点
    double quater_length;
    int update_size_thresh_for_new;
};

//  就像原版的voxelmap,key和octotree关联在一块，这里就是将key的迭代器和octoree关联在一块
struct VoxelValue {
    std::list<VOXEL_KEY>::iterator it;
    std::shared_ptr<OctoTree> tree;
};

// 体素的状态，被更新还是插入
enum class VOXEL_STATE {
    INSERT,
    UPDATE,
};

struct VoxelGrid {
    VOXEL_STATE type;
    std::list<VOXEL_KEY>::iterator it;
    std::vector<PointWithCov> points;
};

using FeatMap = std::unordered_map<VOXEL_KEY, VoxelValue, VOXEL_KEY::Hasher>;
using Submap = std::unordered_map<VOXEL_KEY, VoxelGrid, VOXEL_KEY::Hasher>;

class VoxelMap {
   public:
    VoxelMap(double _voxel_size, int _max_layer, std::vector<int> &_update_size_threshes, int _max_point_thresh,
             double _plane_thresh, int _capacity = 5000000)
        : voxel_size(_voxel_size),
          max_layer(_max_layer),
          max_layer_points(_update_size_threshes),
          max_point_thresh(_max_point_thresh),
          plane_thresh(_plane_thresh),
          capacity(_capacity) {
        feat_map.clear();
        sub_map.clear();
        cache.clear();
    }

    void insert(const std::vector<PointWithCov> &input_points);

    void pack(const std::vector<PointWithCov> &input_points);

    VOXEL_KEY calc_index(const Eigen::Vector3d &point);

    void buildResidual(ResidualData &info, std::shared_ptr<OctoTree> oct_tree);

   public:
    FeatMap feat_map;
    Submap sub_map;
    std::list<VOXEL_KEY> cache;
    int capacity;                       // 保存的体素个数
    double plane_thresh;                // 平面阈值
    int max_point_thresh;               // 体素中点的最大更新个数
    std::vector<int> max_layer_points;  // 每层开始初始化平面的阈值
    int max_layer;                      // 层数
    double voxel_size;                  // 体素大小
};
}  // namespace slam