#include "odom_matcher/ndt_odom_matcher.hh"
#include <pcl/common/transforms.h>
#include <memory>
#include "math_utils.hh"
#include "odom_matcher/inc_ndt.hh"
#include "sensors/lidar.hh"

namespace slam {
NdtOdomMatcher::NdtOdomMatcher(const std::shared_ptr<IESKF>& ieskf) {
    IncNDT::Options options;
    inc_ndt_ptr_ = std::make_shared<IncNDT>(options);
    LOG_INFO("IncNdt OdomMatcher");
    ieskf_ptr = ieskf;
}
NdtOdomMatcher::~NdtOdomMatcher() {
}

void NdtOdomMatcher::AddCloud(const PointCloudPtr& cloud) {
    current_cloud_.reset(new PointCloud);
    current_cloud_ = cloud;
}

void NdtOdomMatcher::Align() {
    // 第一帧
    if (first_scan) {
        inc_ndt_ptr_->AddCloud(current_cloud_);
        first_scan = false;
        return;
    }
    inc_ndt_ptr_->SetInputCloud(current_cloud_);
    ieskf_ptr->UpdateUsingCustomObserve(
        [this](const SE3& pose, Eigen::Matrix<double, 18, 18>& HTVH, Eigen::Matrix<double, 18, 1>& HTVr) {
            inc_ndt_ptr_->ComputeResidualAndJacobians(pose, HTVH, HTVr);
        });
    auto current_pose = ieskf_ptr->GetNominalSE3();
    SE3 delta_pose = last_pose_.inverse() * current_pose;
    // 关键帧
    if (delta_pose.translation().norm() > 1.0 || delta_pose.so3().log().norm() > math::kDEG2RAD * 10.0) {
        // 将点云转换到世界坐标系
        PointCloudPtr cloud_world(new PointCloud);
        pcl::transformPointCloud(*current_cloud_, *cloud_world, current_pose.matrix());
        // 放到ndt中
        inc_ndt_ptr_->AddCloud(cloud_world);
        last_pose_ = current_pose;
    }
}
}  // namespace slam