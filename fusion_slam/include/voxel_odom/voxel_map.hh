#pragma once
#include <unordered_map>
#include <vector>
#include "common/eigen_type.hh"
#define HASH_P 116101
#define MAX_N 10000000000

static int plane_id = 0;

namespace slam {
// 体素的坐标定义
class VOXEL_KEY {
   public:
    //    体素化的坐标
    int64_t x;
    int64_t y;
    int64_t z;

    VOXEL_KEY(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {
    }
    // 重载==
    bool operator==(const VOXEL_KEY& other) const {
        return (x == other.x) && (y == other.y) && (z == other.z);
    }
};
// 对VOXEL_KEY的hash方法

// point with cov
struct PointWithCov {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D point;
    M3D cov;
};

// plane的定义
struct Plane {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D center;  // 平面中心
    V3D nomral;  // 法向量
    V3D x_nomral;
    V3D y_nomral;
    M3D convariance;
    Mat6d plane_cov;
    float radius = 0;  // 在某个半径内
    int id;            // 平面id
    float d = 0;
    int point_size = 0;  //
    bool is_plane = false;
    bool is_init = false;
    // is_update and last_update_points_size are only for publish plane
    bool is_update = false;
    int last_update_points_size = 0;
    bool update_enable = true;  // 在没达到稳定的点个数之前，都是允许被更新的
};

// 点到面的相关结构体
struct ptpl {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D point;        // lidar坐标系下的点
    V3D point_world;  // 世界坐标下的点
    V3D nomral;       // 平面法向量
    V3D center;       // 平面中心
    Mat6d plane_cov;
    double d;  // 点到平面的距离
    int layer;
    M3D cov_lidar;
};

// 八叉树的类
class OctoTree {
   public:
    OctoTree(int max_layer, int layer, std::vector<int> layer_point_size, int max_point_size, int max_cov_points_size,
             float planer_threshold)
        : max_layer_(max_layer),
          layer_(layer),
          layer_point_size_(layer_point_size),
          max_points_size_(max_point_size),
          max_cov_points_size_(max_cov_points_size),
          planer_threshold_(planer_threshold) {
        temp_points_.clear();
        octo_state_ = 0;
        new_points_num_ = 0;
        all_points_num_ = 0;
        // 大于5个点之后就好更新平面
        update_size_threshold_ = 5;
        init_octo_ = false;
        // 允许更新
        update_enable_ = true;
        update_cov_enable_ = true;
        // 每层更新平面的阈值
        max_plane_update_threshold_ = layer_point_size_[layer_];
        // 初始化子节点为空节点
        for (int i = 0; i < 8; ++i) {
            leaves_[i] = nullptr;
        }
        plane_ptr_ = new Plane;
    }

    void init_octo_tree();
    void UpdateOctoTree(const PointWithCov& pv);

   public:
    int max_layer_;                          //最大层数
    int layer_;                              //当前层数
    std::vector<int> layer_point_size_;      //每层开始估计平面的点数阈值
    int max_points_size_;                    //
    int max_cov_points_size_;                //
    double planer_threshold_;                // 平面阈值
    Plane* plane_ptr_;                       // 平面
    OctoTree* leaves_[8];                    // 八叉树结构，
    double voxel_center_[3];                 // voxel的中心，方便后面切割用
    double quater_length_;                   // 体素的四份之一size
    int max_plane_update_threshold_;         //
    int update_size_threshold_;              //
    int all_points_num_;                     //
    int new_points_num_;                     // 新增的点数
    int octo_state_;                         // 树节点能否继续切割
    bool init_octo_;                         // 八叉树节点是否被初始化
    bool update_cov_enable_;                 //
    bool update_enable_;                     // 当平面点到达一定阈值之后，就不需要更新平面了
    std::vector<PointWithCov> temp_points_;  // all points in an octo tree
    std::vector<PointWithCov> new_points_;   // new points in an octo tree
};

void buildVoxelMap(const std::vector<PointWithCov>& input_points, const float voxel_size, const int max_layer,
                   const std::vector<int>& layer_point_size, const int max_points_size, const int max_cov_points_size,
                   const float plane_thresh, std::unordered_map<VOXEL_KEY, OctoTree*>& feat_map);
void updateVoxelMapOMP(const std::vector<PointWithCov>& input_points, const float voxel_size, const int max_layer,
                       const std::vector<int>& layer_point_size, const int max_points_size,
                       const int max_cov_points_size, const float plane_thresh,
                       std::unordered_map<VOXEL_KEY, OctoTree*>& feat_map);
}  // namespace slam

namespace std {
template <>
struct hash<slam::VOXEL_KEY> {
    int64_t operator()(const slam::VOXEL_KEY& s) {
        using std::hash;
        using std::size_t;
        return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
    }
};

}  // namespace std