#include "loop_closure.hh"
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <vector>
#include "sensors/lidar.hh"

namespace slam {

void LoopClosure::Init() {
    LOG_INFO("LoopClosure Init");
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    isam2_ = std::make_shared<gtsam::ISAM2>(isam2_params);
    voxel_filter_.reset(new pcl::VoxelGrid<PointType>);
    voxel_filter_->setLeafSize(loop_params.submap_resolution, loop_params.submap_resolution,
                               loop_params.submap_resolution);
    cloud_pose_history_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pose_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    icp_.reset(new pcl::IterativeClosestPoint<PointType, PointType>);
    icp_->setMaxCorrespondenceDistance(100);
    icp_->setMaximumIterations(50);
    icp_->setTransformationEpsilon(1e-6);
    icp_->setEuclideanFitnessEpsilon(1e-6);
    icp_->setRANSACIterations(0);
}
void LoopClosure::LoopCheck() {
    // 如果位姿为空
    if (temp_poses_.empty()) {
        return;
    }
    // 最新的位姿索引
    int current_index = temp_poses_.size() - 1;
    int prefixe_index = -1;  // 检测到可能回环的所以
    // 遍历位姿
    cloud_pose_history_->clear();
    cloud_pose_history_->reserve(temp_poses_.size());
    for (const auto& pose : temp_poses_) {
        pcl::PointXYZ p;
        p.x = pose.global_pos(0);
        p.y = pose.global_pos(1);
        p.z = pose.global_pos(2);
        cloud_pose_history_->push_back(p);
    }
    pose_kdtree_->setInputCloud(cloud_pose_history_);
    // 接下来就是半径搜索，找到相近的位姿，判断是否为回环
    std::vector<int> ids;
    std::vector<float> dist2;
    pose_kdtree_->radiusSearch(cloud_pose_history_->back(), loop_params.loop_pose_search_thresh, ids, dist2, 0);
    // 遍历得到的索引
    for (int i = 0; i < ids.size(); ++i) {
        int id = ids[i];
        // 回环的位姿时间戳大于阈值
        if (std::abs(temp_poses_[id].time - temp_poses_[current_index].time) > loop_params.time_thresh) {
            prefixe_index = id;
            break;
        }
    }
    // 不符合的回环下标
    if (prefixe_index == -1 || prefixe_index == current_index ||
        current_index - prefixe_index < loop_params.loop_pose_index_thresh) {
        LOG_INFO("can't find loop closure");
        return;
    }
    // 接下来就是做icp
    // 获取submap
    PointCloudPtr cur_cloud = GetSubmap(temp_poses_, shared_data_ptr_->cloud_historys, current_index, 0);
    PointCloudPtr submaps =
        GetSubmap(temp_poses_, shared_data_ptr_->cloud_historys, prefixe_index, loop_params.submap_search_num);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*cur_cloud, *cur_cloud_xyzi);
    pcl::copyPointCloud(*submaps, *submap_cloud_xyzi);
    cur_cloud = addNorm(cur_cloud_xyzi);
    submaps = addNorm(submap_cloud_xyzi);
    icp_->setInputSource(cur_cloud);
    icp_->setInputTarget(submaps);
    PointCloudPtr aligned(new PointCloud);

    icp_->align(*aligned, Eigen::Matrix4f::Identity());

    float score = icp_->getFitnessScore();

    if (!icp_->hasConverged() || score > loop_params.loop_icp_thresh) return;
    LOG_INFO("Detected LOOP: {} {} {}", prefixe_index, current_index, score);

    shared_data_ptr_->loop_history.emplace_back(prefixe_index, current_index);
    loop_found_ = true;

    Eigen::Matrix4d T_pre_cur = icp_->getFinalTransformation().cast<double>();
    Eigen::Matrix3d R12 = temp_poses_[prefixe_index].global_rot.transpose() * T_pre_cur.block<3, 3>(0, 0) *
                          temp_poses_[current_index].global_rot;
    Eigen::Vector3d t12 = temp_poses_[prefixe_index].global_rot.transpose() *
                          (T_pre_cur.block<3, 3>(0, 0) * temp_poses_[current_index].global_pos +
                           T_pre_cur.block<3, 1>(0, 3) - temp_poses_[prefixe_index].global_pos);
    shared_data_ptr_->loop_pairs.emplace_back(prefixe_index, current_index, score, R12, t12);
}

void LoopClosure::AddOdomFactor() {
    for (int i = previous_index; i < lastest_index_; ++i) {
        Pose6D& p_head = temp_poses_[i];
        Pose6D& p_next = temp_poses_[i + 1];
        // 固定第一个里程计
        if (i == 0) {
            initialized_estimate_.insert(i,
                                         gtsam::Pose3(gtsam::Rot3(p_head.local_rot), gtsam::Point3(p_head.local_pos)));
            gtsam::noiseModel::Diagonal::shared_ptr noise =
                gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
            // 先验
            gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
                i, gtsam::Pose3(gtsam::Rot3(p_head.local_rot), gtsam::Point3(p_head.local_pos)), noise));
        }
        initialized_estimate_.insert(i + 1,
                                     gtsam::Pose3(gtsam::Rot3(p_next.local_rot), gtsam::Point3(p_next.local_pos)));
        // 里程计约束
        M3D R12 = p_head.local_rot.transpose() * p_next.local_rot;
        V3D t12 = p_head.local_rot.transpose() * (p_next.local_pos - p_head.local_pos);
        gtsam::noiseModel::Diagonal::shared_ptr noise =
            gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
        gtsam_graph_.add(
            gtsam::BetweenFactor<gtsam::Pose3>(i, i + 1, gtsam::Pose3(gtsam::Rot3(R12), gtsam::Point3(t12)), noise));
    }
    previous_index = lastest_index_;
}
void LoopClosure::AddLoopFactor() {
    if (!loop_found_) {
        return;
    }
    if (shared_data_ptr_->loop_pairs.empty()) {
        return;
    }
    // 遍历所有的回环部分
    for (LoopPair& loop_pair : shared_data_ptr_->loop_pairs) {
        // 根据分数作为噪声
        gtsam::Pose3 pose_between(gtsam::Rot3(loop_pair.diff_rot), gtsam::Point3(loop_pair.diff_trans));
        gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            loop_pair.pre_index, loop_pair.cur_index, pose_between,
            gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * loop_pair.score)));
    }
    shared_data_ptr_->loop_pairs.clear();
}
void LoopClosure::OptimizeAndUpdate() {
    isam2_->update(gtsam_graph_, initialized_estimate_);
    isam2_->update();
    if (loop_found_) {
        isam2_->update();
        isam2_->update();
        isam2_->update();
        isam2_->update();
        isam2_->update();
        loop_found_ = false;
    }
    gtsam_graph_.resize(0);
    initialized_estimate_.clear();
    // 优化后的位姿
    optimized_estimate_ = isam2_->calculateBestEstimate();
    gtsam::Pose3 lastest_estimate = optimized_estimate_.at<gtsam::Pose3>(lastest_index_);
    temp_poses_[lastest_index_].global_rot = lastest_estimate.rotation().matrix().cast<double>();
    temp_poses_[lastest_index_].global_pos = lastest_estimate.translation().cast<double>();
    //  get offset
    M3D offset_rot;
    V3D offset_trans;
    temp_poses_[lastest_index_].getOffset(offset_rot, offset_trans);
    // 修改所有的位姿
    shared_data_ptr_->shared_data_mutex.lock();
    int current_size = shared_data_ptr_->key_poses.size();
    LOG_INFO("all pose size:{}", current_size);
    shared_data_ptr_->offset_rot = offset_rot;
    shared_data_ptr_->offset_trans = offset_trans;
    for (int i = 0; i < lastest_index_; ++i) {
        gtsam::Pose3 temp_pose = optimized_estimate_.at<gtsam::Pose3>(i);
        shared_data_ptr_->key_poses[i].global_rot = temp_pose.rotation().matrix().cast<double>();
        shared_data_ptr_->key_poses[i].global_pos = temp_pose.translation().cast<double>();
    }
    for (int i = lastest_index_; i < current_size; i++) {
        shared_data_ptr_->key_poses[i].addOffset(offset_rot, offset_trans);
    }
    shared_data_ptr_->shared_data_mutex.unlock();
}

void LoopClosure::operator()() {
    // while (ros::ok()) {
    //     loop_rate_->sleep();
    //     if (!loop_params.active) {
    //         continue;
    //     }
    //     // keypose 为空或者小于阈值
    //     if (shared_data_ptr_->key_poses.empty()) {
    //         LOG_INFO("key_pose_empty");
    //         continue;
    //     }

    //     if (shared_data_ptr_->key_poses.size() < loop_params.loop_pose_search_thresh) {
    //         LOG_INFO("key_pose:{} < pose_thresh:{}", shared_data_ptr_->key_poses.size(),
    //                  loop_params.loop_pose_search_thresh);
    //         continue;
    //     }
    //     // 没有添加keypose
    //     if (!shared_data_ptr_->key_pose_added) {
    //         continue;
    //     }
    //     // 清空标志位
    //     shared_data_ptr_->key_pose_added = false;
    //     {
    //         // 加锁
    //         std::lock_guard<std::mutex> lck(shared_data_ptr_->shared_data_mutex);
    //         lastest_index_ = shared_data_ptr_->key_poses.size() - 1;
    //         LOG_INFO("lastest_index:{}", lastest_index_);
    //         temp_poses_.clear();
    //         // copy keyposes
    //         temp_poses_.assign(shared_data_ptr_->key_poses.begin(), shared_data_ptr_->key_poses.end());
    //     }
    //     LoopCheck();
    //     AddOdomFactor();
    //     AddLoopFactor();
    //     OptimizeAndUpdate();
    // }
}  // namespace slam
PointCloudPtr LoopClosure::GetSubmap(const std::vector<Pose6D>& pose_list,
                                     const std::vector<PointCloudPtr>& cloud_historty, int start_index,
                                     int cloud_nums) {
    LOG_INFO("cloud_history:{}", cloud_historty.size());
    PointCloudPtr cloud(new PointCloud);
    int max_size = pose_list.size();
    // 取前面和后面的多少帧拼接起来
    int min_index = std::max(0, start_index - cloud_nums);
    int max_index = std::min(max_size - 1, start_index + cloud_nums);
    LOG_INFO("min_index:{}, max_index:{}", min_index, max_index);
    for (int i = min_index; i <= max_index; ++i) {
        const Pose6D pose = pose_list[i];
        Eigen::Matrix<double, 4, 4> T = Eigen::Matrix<double, 4, 4>::Identity();
        T.block<3, 3>(0, 0) = pose.global_rot;
        T.block<3, 1>(0, 3) = pose.global_pos;
        PointCloudPtr tmp_cloud(new PointCloud);
        pcl::transformPointCloud(*cloud_historty[pose.index], *tmp_cloud, T);
        *cloud += *tmp_cloud;
    }
    LOG_INFO("cloud_size:{}", cloud->size());
    voxel_filter_->setInputCloud(cloud);
    voxel_filter_->filter(*cloud);
    return cloud;
}
}  // namespace slam