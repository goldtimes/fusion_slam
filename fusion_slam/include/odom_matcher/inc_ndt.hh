#pragma once

#include <list>
#include <unordered_map>
#include <vector>
#include "common/eigen_type.hh"
#include "common/lidar_point_type.hh"
#include "common/logger.hpp"
#include "math_utils.hh"
#include "sensors/lidar.hh"

namespace slam {
/**
    增量的ndt配准算法
 */
class IncNDT {
   public:
    enum class NearbyType {
        CENTER,
        NEARBY6,
    };

    // ndt的配置类
    struct Options {
        Options(){};
        // 体素大小
        double voxel_size_ = 1.0;
        double inv_voxel_size = 1.0;
        // 体素中最小的点云个数以及最大的点数
        int min_pts_in_voxel_;
        int max_pts_in_voxel_;
        // 附近的搜索个数
        NearbyType nearby_type_ = NearbyType::NEARBY6;
        // 维护的体素个数
        int max_capacity_ = 1e6;
        // 收敛的判断
        double eps_ = 1e-3;
        // 迭代次数
        int max_iteration_ = 4;
        double res_outlier_th_ = 5.0;
    };
    // 定义体素的结构体
    struct VoxelData {
        VoxelData() {
        }
        VoxelData(const Vec3d& pt) {
            pts_.push_back(pt);
            num_pts_++;
        }

        void AddPoint(const Vec3d& pt) {
            pts_.emplace_back(pt);
            if (!ndt_estimated_) {
                num_pts_++;
            }
        }

        // 存放体素中的点云
        std::vector<Vec3d> pts_;
        // 均值
        Vec3d mu_ = Vec3d::Zero();
        // 协方差
        Mat3d cov_ = Mat3d::Zero();
        // 协方差的逆
        Mat3d info = Mat3d::Zero();
        // 保存体素中的点个数
        int num_pts_;
        // ndt是否已经估计
        bool ndt_estimated_ = false;
    };

    using KeyType = Eigen::Matrix<int, 3, 1>;
    using KeyAndType = std::pair<KeyType, VoxelData>;
    IncNDT();
    IncNDT(Options options = Options());

    // 添加点云到体素中
    void AddCloud(const PointCloudPtr& cloud);
    // 设置等待配准的点云
    void SetInputCloud(const PointCloudPtr& cloud);
    // lm/gauss-newton的方法来求解
    bool AlignNDT(const SE3& guess_pose);

    void ComputeResidualAndJacobians(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr);

   private:
    void GenerateNearyGrids();
    // 添加点之后需要更新体素信息
    void UpdateVoxel(VoxelData& voxel_data);

    Vec3d ToVec3d(const PointXYZIRT& point) {
        return point.getVector3fMap().cast<double>();
    }

    PointXYZIRT Vec3dToPoint(const Vec3d& eigen_pt) {
        PointXYZIRT point;
        point.x = eigen_pt[0];
        point.y = eigen_pt[1];
        point.z = eigen_pt[2];
        return point;
    }
    // eigen doule to int
    template <typename S, int n>
    Eigen::Matrix<int, n, 1> CastToInt(const Eigen::Matrix<S, n, 1>& value) {
        return value.array().template round().template cast<int>();
    }

   private:
    Options options_;
    // 附近的体素
    std::vector<KeyType> nearby_grids_;
    // 链表，存储体素，需要删除旧的体素
    std::list<KeyAndType> voxels_lists_;
    // grids 数据, 存储的key和list中迭代器位置
    std::unordered_map<KeyType, std::list<KeyAndType>::iterator, math::hash_vec<3>> grids_;
    PointCloudPtr current_cloud_;
    bool first_scan_ = true;
};
}  // namespace slam