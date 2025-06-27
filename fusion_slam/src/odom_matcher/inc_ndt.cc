#include "odom_matcher/inc_ndt.hh"
#include <algorithm>
#include <cmath>
#include <execution>
#include <set>
#include <vector>
#include "common/eigen_type.hh"
#include "math_utils.hh"
#include "sophus/so3.hpp"

namespace slam {
IncNDT::IncNDT() {
    options_.inv_voxel_size = 1.0 / options_.voxel_size_;
    GenerateNearyGrids();
}
IncNDT::IncNDT(Options options) : options_(options) {
    options_.inv_voxel_size = 1.0 / options_.voxel_size_;
    GenerateNearyGrids();
}

void IncNDT::GenerateNearyGrids() {
    if (options_.nearby_type_ == NearbyType::NEARBY6) {
        // clang-format off
        nearby_grids_ = {KeyType(0, 0, 0), KeyType(1, 0, 0), KeyType(-1, 0, 0), 
                                   KeyType(0, 1, 0), KeyType(0, -1, 0), KeyType(0, 0, 1), 
                                   KeyType(0, 0, -1)};
        // clang-format on
    } else {
        nearby_grids_.emplace_back(KeyType::Zero());
    }
}

// 添加点云到体素中
void IncNDT::AddCloud(const PointCloudPtr& cloud) {
    // 遍历所有的点云，记录哪些体素是要更新的。之后再用for_each去更新栅格
    std::set<KeyType, math::less_vec<3>> active_grids;
    for (const auto& pt : cloud->points) {
        auto pt_eigen = ToVec3d(pt);
        // 计算key
        auto key = CastToInt(Vec3d(pt_eigen * options_.inv_voxel_size));
        auto iter = grids_.find(key);
        if (iter != grids_.end()) {
            // 已经存在该体素了
            // 将该点添加到体素中
            iter->second->second.AddPoint(pt_eigen);
            // 因为你这个体素被更新了，所以我们要放到list的最前面
            // 将链表 x 中由迭代器 i 指向的元素转移到当前链表的 position 位置之前。
            // void splice(iterator position, list& x, iterator i);
            voxels_lists_.splice(voxels_lists_.begin(), voxels_lists_, iter->second);
            iter->second = voxels_lists_.begin();
        } else {
            // 在链表中添加
            voxels_lists_.push_front({key, {pt_eigen}});
            grids_.insert({key, voxels_lists_.begin()});
            // 删除旧的体素信息
            if (voxels_lists_.size() > options_.max_capacity_) {
                voxels_lists_.pop_back();
                grids_.erase(voxels_lists_.back().first);
            }
        }
        active_grids.emplace(key);
    }
    // 更新体素中的统计量
    std::for_each(std::execution::par_unseq, active_grids.begin(), active_grids.end(),
                  [this](const auto& key) { UpdateVoxel(grids_[key]->second); });
    first_scan_ = false;
}
// 设置等待配准的点云
void IncNDT::SetInputCloud(const PointCloudPtr& cloud) {
    current_cloud_ = cloud;
}
// lm/gauss-newton的方法来求解
bool IncNDT::AlignNDT(const SE3& guess_pose) {
    return true;
}

void IncNDT::UpdateVoxel(VoxelData& voxel_data) {
    if (first_scan_) {
        // 体素中的点 > 1
        if (voxel_data.pts_.size() > 1) {
            // 计算均值和协方差
            math::ComputeMeanAndCov(voxel_data.pts_, voxel_data.mu_, voxel_data.cov_,
                                    [this](const Vec3d& p) { return p; });
            voxel_data.info = (voxel_data.cov_ + Mat3d::Identity() * 1e-3).inverse();
        } else {
            voxel_data.mu_ = voxel_data.pts_[0];
            voxel_data.info = Mat3d::Identity() * 1e2;
        }
        voxel_data.ndt_estimated_ = true;
        // 估计完均值和协方差不需要保存之前的点，清除就行
        voxel_data.pts_.clear();
        return;
    }
    if (voxel_data.ndt_estimated_ && voxel_data.num_pts_ > options_.max_pts_in_voxel_) {
        // 不需要更新
        return;
    }
    // 没有估计ndt且大于min_pts_in_voxel_
    if (!voxel_data.ndt_estimated_ && voxel_data.pts_.size() > options_.min_pts_in_voxel_) {
        // 新增的voxel
        math::ComputeMeanAndCov(voxel_data.pts_, voxel_data.mu_, voxel_data.cov_, [this](const Vec3d& p) { return p; });
        // + Mat3d::Identity() * 1e-3 防止为0
        voxel_data.info = (voxel_data.cov_ + Mat3d::Identity() * 1e-3).inverse();
        voxel_data.ndt_estimated_ = true;
        voxel_data.pts_.clear();

    } else if (voxel_data.ndt_estimated_ && voxel_data.pts_.size() > options_.min_pts_in_voxel_) {
        // 估计了旧的体素，但是 min_pts_in_voxel_ < 点数 < max_pts_in_voxel_
        // 计算均值和协方差
        Vec3d cur_mu, new_mu;
        Mat3d cur_cov, new_cov;
        // 计算新增点的均值和协方差
        math::ComputeMeanAndCov(voxel_data.pts_, cur_mu, cur_cov, [this](const Vec3d& p) { return p; });
        math::UpdateMeanAndCov(voxel_data.num_pts_, voxel_data.pts_.size(), voxel_data.mu_, voxel_data.cov_, cur_mu,
                               cur_cov, new_mu, new_cov);
        // 更新均值和协方差
        voxel_data.mu_ = new_mu;
        voxel_data.cov_ = new_cov;
        voxel_data.num_pts_ += voxel_data.pts_.size();
        voxel_data.pts_.clear();

        // check info
        Eigen::JacobiSVD svd(voxel_data.cov_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Vec3d lambda = svd.singularValues();
        if (lambda[1] < lambda[0] * 1e-3) {
            lambda[1] = lambda[0] * 1e-3;
        }

        if (lambda[2] < lambda[0] * 1e-3) {
            lambda[2] = lambda[0] * 1e-3;
        }

        Mat3d inv_lambda = Vec3d(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
        voxel_data.info = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
    }
}

// 根据传入的点云以及pose和之前构建的ntd体素信息进行配准，计算残差和雅可比矩阵
void IncNDT::ComputeResidualAndJacobians(const SE3& pose, Mat18d& HTVH, Vec18d& HTVr) {
    // 根据搜索的体素来决定每个点的残差个数
    int num_residual_per_points = options_.nearby_type_ == NearbyType::CENTER ? 1 : 7;
    // 下标
    std::vector<int> index(current_cloud_->size());
    for (int i = 0; i < index.size(); ++i) {
        index[i] = i;
    }
    int total_size = index.size() * num_residual_per_points;
    std::vector<bool> effect_points(total_size, false);
    std::vector<Eigen::Matrix<double, 3, 18>> jacobians(total_size);
    std::vector<Vec3d> errors(total_size);
    std::vector<Mat3d> infos(total_size);
    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](int idx) {
        auto eigen_pt = ToVec3d(current_cloud_->points[idx]);
        //   转换到guess 位姿上
        auto trans_pt = pose * eigen_pt;
        // 计算点落在的体素中
        auto key = CastToInt(Vec3d(trans_pt * options_.inv_voxel_size));
        // 遍历周围的体素
        for (int i = 0; i < nearby_grids_.size(); ++i) {
            Vec3i real_key = key + nearby_grids_[i];
            auto it = grids_.find(real_key);
            // 0~6,7~11
            int real_index = idx * num_residual_per_points + i;
            // 找到了该体素，并且已经估计了协方差
            if (it != grids_.end() && it->second->second.ndt_estimated_) {
                // 计算残差
                VoxelData data = it->second->second;
                Vec3d e = trans_pt - data.mu_;
                double res = e.transpose() * data.info * e;
                // 残差值太大，拒绝
                if (std::isnan(res) || res > options_.res_outlier_th_) {
                    effect_points[real_index] = false;
                    continue;
                }
                // jacobian矩阵
                Eigen::Matrix<double, 3, 18> J;
                J.setZero();
                J.block<3, 3>(0, 0) = Mat3d::Identity();                                   // 对p
                J.block<3, 3>(0, 6) = -pose.so3().matrix() * Sophus::SO3d::hat(eigen_pt);  // 对R
                jacobians[real_index] = J;
                effect_points[real_index] = true;
                errors[real_index] = e;
                infos[real_index] = data.info;
            } else {
                effect_points[real_index] = false;
            }
        }
    });

    // 计算H矩阵 和 error
    double total_res = 0;
    int effect_nums = 0;
    HTVH.setZero();
    HTVr.setZero();
    const double info_ratio = 0.01;  // 每个点反馈的info因子
    for (int idx = 0; idx < effect_points.size(); ++idx) {
        if (!effect_points[idx]) {
            continue;
        }
        total_res += errors[idx].transpose() * infos[idx] * errors[idx];
        effect_nums++;
        HTVH += jacobians[idx].transpose() * infos[idx] * jacobians[idx] * info_ratio;
        HTVr += -jacobians[idx].transpose() * infos[idx] * errors[idx] * info_ratio;
    }
    LOG_INFO("IncNdt get effective num:{}, total_res:{}", effect_nums, total_res);
}
}  // namespace slam