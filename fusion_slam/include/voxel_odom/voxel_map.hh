#pragma once
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
    }

   private:
    int max_layer_;                      //最大层数
    int layer_;                          //当前层数
    std::vector<int> layer_point_size_;  //每层开始估计平面的点数阈值
    int capacity_;                       //最大的体素个数
    int max_points_size_;                //
    int max_cov_points_size_;            //
    double planer_threshold_;            // 平面阈值
};

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