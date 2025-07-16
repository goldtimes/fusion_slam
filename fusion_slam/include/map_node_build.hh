/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-08 23:14:53
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-13 21:45:09
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/map_node_build.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <thread>
#include <vector>
#include "common/common_lib.hh"
#include "common/logger.hh"
#include "fastlio_odom/fastlio_odom.hh"
#include "lidar_process.hh"
#include "loop_closure.hh"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "voxel_odom/voxel_odom.hh"
namespace slam {
class MapBuildNode {
   public:
    struct MapBuildNodeConfig {
        std::string global_frame_;
        std::string local_frame_;
        std::string body_frame_;
        std::string imu_topic_;
        std::string lidar_topic_;
        std::string lidar_type_;
        double det_range;
        double cube_len;
        double map_resolution;
        double move_thresh;
        bool align_gravity;
        M3D imu_ext_rot;
        V3D imu_ext_pos;
        bool use_voxel_map;
    };

   public:
    MapBuildNode(const ros::NodeHandle& nh);
    void Run();

   private:
    void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void standard_pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg);
    void livox_callback(const livox_ros_driver2::CustomMsg::ConstPtr& lidar_msg);
    void encoder_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void gnss_callback(const sensor_msgs::NavSatFix::ConstPtr& gnss_msg);

    void load_params();
    void init_sub_pub();

    bool SyncPackage(MeasureGroup& sync_package);

    void publishCloud(ros::Publisher& publisher, const sensor_msgs::PointCloud2& cloud_to_pub) {
        if (publisher.getNumSubscribers() == 0) return;
        publisher.publish(cloud_to_pub);
    }

    void publishOdom(const nav_msgs::Odometry& odom_to_pub) {
        if (odom_pub_.getNumSubscribers() == 0) return;
        odom_pub_.publish(odom_to_pub);
    }

    Eigen::Vector3d rotate2rpy(Eigen::Matrix3d& rot) {
        double roll = std::atan2(rot(2, 1), rot(2, 2));
        double pitch = asin(-rot(2, 0));
        double yaw = std::atan2(rot(1, 0), rot(0, 0));
        return Eigen::Vector3d(roll, pitch, yaw);
    }

    void addKeypose();
    void publishLocalPath();
    void publishGlobalPath();
    void publishLoopMark();

   private:
    ros::NodeHandle nh_;
    MapBuildNodeConfig config_;
    LidarProcess::LidarProcessConfig lidar_process_config_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    ros::Subscriber imu_sub_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber gnss_sub_;
    ros::Subscriber encoder_sub_;

    ros::Publisher odom_pub_;
    ros::Publisher body_cloud_pub_;
    ros::Publisher world_cloud_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher global_path_pub_;
    ros::Publisher loop_mark_pub_;

    std::deque<PointCloudPtr> lidar_queque_;
    std::deque<double> lidar_time_queue_;
    std::deque<IMUData> imu_queque_;
    std::deque<Encoder> encoder_queque_;
    std::deque<GNSS> gnss_queque_;
    //
    double last_imu_time = -1;
    double last_lidar_time = -1;
    double last_encoder_time = -1;
    double last_gnss_time = -1;

    std::shared_ptr<LidarProcess> lidar_process_ptr_;
    std::shared_ptr<ros::Rate> local_rate_;
    std::shared_ptr<ros::Rate> loop_rate_;

    MeasureGroup sync_package;
    bool lidar_pushed = false;
    double lidar_mean_scantime = 0.0;
    int scan_num = 0;
    double current_time_;
    state_ikfom current_state_;
    std::shared_ptr<FastlioOdom> fastlio_odom_ptr_;
    FastlioOdom::FastlioOdomConfig fastlio_odom_config_;

    std::shared_ptr<VoxelOdom> voxel_odom_ptr_;
    VoxelOdom::VoxelOdomConfig voxel_odom_config_;

    std::mutex mtx;

    // 回环检测 就是开启一个优化线程，接受odom传递过来的位姿信息，以及将点云保存下来
    // 然后线程不断地check是否有检测到回环，如果检测到回环，将当前位姿的点云拼接起来，并进行icp.如果匹配得分ok,那么就是回环了
    // 然后进行优化位姿
    LoopClosure loop_closure_;
    std::shared_ptr<std::thread> loop_closure_thread_;
    std::shared_ptr<SharedData> shared_data;
};
}  // namespace slam