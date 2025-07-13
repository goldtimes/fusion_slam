#pragma once
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include "common/eigen_type.hh"
#include "ros/ros.h"
#include "sensors/lidar.hh"

// gtsam
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include "common/logger.hh"

namespace slam {

struct LoopPair {
    LoopPair(int p_index, int c_index, double s, const M3D& diff_rotation, const V3D& diff_translation)
        : pre_index(p_index), cur_index(c_index), score(s), diff_rot(diff_rotation), diff_trans(diff_translation) {
    }
    M3D diff_rot;    // R12
    V3D diff_trans;  // t12
    // 匹配的分数
    double score;
    // 找到回环的索引
    int pre_index;
    int cur_index;
};

struct LoopClosureParams {
    double rad_thresh = 0.4;                // 角度增量大于这个阈值认为是关键帧
    double dist_thresh = 2.5;               // 距离增量大于这个阈值认为是关键帧
    double time_thresh = 30.0;              // 避免过早的回环，需要对关键帧时间进行判断
    double loop_pose_search_thresh = 10.0;  // 半径搜索范围
    int loop_pose_index_thresh = 5;         // 认为回环的index 要相距够远
    double submap_resolution = 0.2;         // 对子图进行降采样
    double loop_icp_thresh = 0.3;           // icp 匹配上的阈值
    bool active = true;
    int submap_search_num = 20;
};

// Pose 用来保存局部位姿和经过闭环检测的位姿
struct Pose6D {
    Pose6D(int i, double t, Eigen::Matrix3d lr, Eigen::Vector3d lp) : index(i), time(t), local_rot(lr), local_pos(lp) {
    }
    void SetGlobalPose(const M3D& gr, const V3D& gp) {
        global_rot = gr;
        global_pos = gp;
    }
    // 对局部位姿进行优化后加上优化后的偏移量
    void addOffset(const Eigen::Matrix3d& offset_rot, const Eigen::Vector3d& offset_pos) {
        global_rot = offset_rot * local_rot;
        global_pos = offset_rot * local_pos + offset_pos;
    }
    void getOffset(Eigen::Matrix3d& offset_rot, Eigen::Vector3d& offset_pos) {
        offset_rot = global_rot * local_rot.transpose();
        offset_pos = -global_rot * local_rot.transpose() * local_pos + global_pos;
    }
    int index;
    double time;
    M3D local_rot;
    V3D local_pos;
    M3D global_rot;
    V3D global_pos;
};

/**
点云以及位姿以及闭环的pair
*/
struct SharedData {
    std::mutex shared_data_mutex;
    bool key_pose_added = false;
    // 历史的位姿
    std::vector<Pose6D> key_poses;
    // 检测到的闭环结果
    std::vector<LoopPair> loop_pairs;
    std::vector<std::pair<int, int>> loop_history;
    std::vector<PointCloudPtr> cloud_historys;
    M3D offset_rot = M3D::Identity();
    V3D offset_trans = V3D::Zero();
};

//  这个类需要传递到线程中，需要重载()函数
class LoopClosure {
   public:
    void Init();
    void LoopCheck();
    void AddOdomFactor();
    void AddLoopFactor();
    void OptimizeAndUpdate();
    void operator()();

    PointCloudPtr GetSubmap(const std::vector<Pose6D>& pose_list, const std::vector<PointCloudPtr>& cloud_historty,
                            int start_index, int cloud_nums);

    void setShared(std::shared_ptr<SharedData> share_data) {
        shared_data_ptr_ = share_data;
    }
    void setRate(const double& rate) {
        loop_rate_ = std::make_shared<ros::Rate>(rate);
    }
    void setRate(std::shared_ptr<ros::Rate> rate) {
        loop_rate_ = rate;
    }
    LoopClosureParams& mutableParams() {
        return loop_params;
    }

    PointCloudPtr addNorm(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZI>);
        searchTree->setInputCloud(cloud);

        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
        normalEstimator.setInputCloud(cloud);
        normalEstimator.setSearchMethod(searchTree);
        normalEstimator.setKSearch(15);
        normalEstimator.compute(*normals);
        PointCloudPtr out(new PointCloud);
        pcl::concatenateFields(*cloud, *normals, *out);
        return out;
    }

   private:
    // 非线性的位姿图优化器
    gtsam::NonlinearFactorGraph gtsam_graph_;
    gtsam::Values initialized_estimate_;
    gtsam::Values optimized_estimate_;
    std::shared_ptr<gtsam::ISAM2> isam2_;
    // icp配准相关
    pcl::VoxelGrid<PointType>::Ptr voxel_filter_;
    // 将位姿存储为点云的格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pose_history_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr pose_kdtree_;
    // icp
    pcl::IterativeClosestPoint<PointType, PointType>::Ptr icp_;
    std::shared_ptr<SharedData> shared_data_ptr_;
    std::shared_ptr<ros::Rate> loop_rate_;
    LoopClosureParams loop_params;
    std::vector<Pose6D> temp_poses_;
    int previous_index = 0;
    int lastest_index_;
    bool loop_found_ = false;
};
}  // namespace slam