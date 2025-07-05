#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include "common/eigen_type.hh"
#include "common/lidar_point_type.hh"
#include "common/logger.hpp"
#include "common/pointcloud_utils.hh"
#include "common_lib.hh"
#include "fastlio_odom/fastkio_ieskf.hh"
#include "fastlio_odom/fastlio_odom.hh"
#include "livox_ros_driver2/CustomMsg.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/time.h"
#include "sensors/imu.hh"
#include "sensors/lidar.hh"
#include "static_imu_init.hh"

namespace slam {

class LIONode {
   public:
    LIONode(const ros::NodeHandle nh);
    ~LIONode() = default;

    void run();

   private:
    void loadParames();

    void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void StandardLidarCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void LivoxLidarCallback(const livox_ros_driver2::CustomMsg::ConstPtr& livox_msg);

    bool sync_package(MeasureGroup& measure);

    void PublishTf(const std::string& parent_frame, const std::string& child_frame, double timestamped);
    void PublishOdom(const std::string& parent_frame, const std::string& child_frame, double timestamped);
    void PublishCloud(const PointCloudPtr& cloud, const std::string& frame, double timestamped);

    void rosIMUtoIMU(const sensor_msgs::Imu::ConstPtr& imu_msgs, IMUData& imu_data, bool is_livox = false,
                     bool has_orientation = false) {
        imu_data.timestamped_ = imu_msgs->header.stamp.toSec();
        if (is_livox) {
            const double gravity = 9.81;
            imu_data.acc_ =
                Eigen::Vector3d(imu_msgs->linear_acceleration.x * 9.81, imu_msgs->linear_acceleration.y * 9.81,
                                imu_msgs->linear_acceleration.z * 9.81);
        }
        imu_data.gyro_ =
            Eigen::Vector3d(imu_msgs->angular_velocity.x, imu_msgs->angular_velocity.y, imu_msgs->angular_velocity.z);
        if (has_orientation) {
            imu_data.orientation.w() = imu_msgs->orientation.w;
            imu_data.orientation.x() = imu_msgs->orientation.x;
            imu_data.orientation.y() = imu_msgs->orientation.y;
            imu_data.orientation.z() = imu_msgs->orientation.z;
        }
    }

    PointCloud::Ptr livox2pcl(const livox_ros_driver2::CustomMsg::ConstPtr& livox_msg) {
        PointCloudPtr cloud(new PointCloud);
        const double min_dist = m_node_config.lidar_min_range * m_node_config.lidar_min_range;
        const double max_dist = m_node_config.lidar_max_range * m_node_config.lidar_max_range;
        int point_num = livox_msg->point_num;
        cloud->reserve(point_num / m_node_config.lidar_filter_num);
        for (int i = 0; i < point_num; i += m_node_config.lidar_filter_num) {
            if ((livox_msg->points[i].line < 4) &&
                ((livox_msg->points[i].tag & 0x30) == 0x10 || (livox_msg->points[i].tag & 0x30) == 0x00)) {
                float x = livox_msg->points[i].x;
                float y = livox_msg->points[i].y;
                float z = livox_msg->points[i].z;
                PointXYZIRT p;
                p.x = x;
                p.y = y;
                p.z = z;
                p.intensity = livox_msg->points[i].reflectivity;
                p.time = livox_msg->points[i].offset_time / 1000000.0f;
                cloud->push_back(p);
            }
        }
        return cloud;
    }

   private:
    ros::NodeHandle nh_;
    // sub
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    // pub
    ros::Publisher body_cloud_pub_;
    ros::Publisher world_cloud_pub_;
    ros::Publisher path_pub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster tf_;
    MeasureGroup measures_;

    LIONodeConfig m_node_config;
    // thead
    std::thread sync_thread_;

    std::mutex imu_mutex_;
    std::mutex lidar_mutex_;
    std::deque<IMUData> imu_queue_;
    std::deque<PointCloud::Ptr> lidar_queue_;
    std::deque<double> lidar_time_queue_;

    double last_imu_time = -1;
    double last_lidar_time = -1;

    bool lidar_pushed_ = false;
    double lidar_mean_scantime_ = 0.0;
    uint64_t scan_num_ = 0;

    std::shared_ptr<fastlio::FastlioIESKF> ieskf_;
    std::shared_ptr<fastlio::FastLioOdom> fastlio_odom_ptr_;
    // tf
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};
}  // namespace slam

slam::LIONode::LIONode(const ros::NodeHandle nh) : nh_(nh) {
    loadParames();
    imu_sub_ = nh_.subscribe(m_node_config.imu_topic, 200, &LIONode::ImuCallback, this);
    lidar_sub_ = m_node_config.livox_msg
                     ? nh_.subscribe(m_node_config.lidar_topic, 10, &LIONode::LivoxLidarCallback, this)
                     : nh_.subscribe(m_node_config.lidar_topic, 10, &LIONode::StandardLidarCallback, this);
    body_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("body_cloud", 10);
    world_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("world_cloud", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("lio_path", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("lio_odom", 10);
    sync_thread_ = std::thread(&LIONode::run, this);
    ieskf_ = std::make_shared<fastlio::FastlioIESKF>();
    fastlio_odom_ptr_ = std::make_shared<fastlio::FastLioOdom>(m_node_config, ieskf_);
}

void slam::LIONode::loadParames() {
    std::string config_path;
    nh_.param<std::string>("config_path", config_path, "");
    LOG_INFO("LOAD FROM YAML CONFIG PATH: {}", config_path);

    YAML::Node config = YAML::LoadFile(config_path);
    if (!config) {
        LOG_INFO("FAIL TO LOAD YAML FILE!");
        return;
    }

    m_node_config.imu_topic = config["imu_topic"].as<std::string>();
    m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
    m_node_config.body_frame = config["body_frame"].as<std::string>();
    m_node_config.world_frame = config["world_frame"].as<std::string>();
    m_node_config.print_time_cost = config["print_time_cost"].as<bool>();
    m_node_config.livox_msg = config["livox_msg"].as<bool>();

    m_node_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
    m_node_config.lidar_min_range = config["lidar_min_range"].as<double>();
    m_node_config.lidar_max_range = config["lidar_max_range"].as<double>();
    m_node_config.scan_resolution = config["scan_resolution"].as<double>();
    m_node_config.map_resolution = config["map_resolution"].as<double>();
    m_node_config.cube_len = config["cube_len"].as<double>();
    m_node_config.det_range = config["det_range"].as<double>();
    m_node_config.move_thresh = config["move_thresh"].as<double>();
    m_node_config.na = config["na"].as<double>();
    m_node_config.ng = config["ng"].as<double>();
    m_node_config.nba = config["nba"].as<double>();
    m_node_config.nbg = config["nbg"].as<double>();

    m_node_config.imu_init_num = config["imu_init_num"].as<int>();
    m_node_config.near_search_num = config["near_search_num"].as<int>();
    m_node_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
    m_node_config.gravity_align = config["gravity_align"].as<bool>();
    m_node_config.esti_il = config["esti_il"].as<bool>();
    std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
    std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
    m_node_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
    m_node_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6],
        r_il_vec[7], r_il_vec[8];
    m_node_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
}

void slam::LIONode::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    IMUData imu_data;
    std::lock_guard<std::mutex> lock(imu_mutex_);
    double curr_imu_time = imu_msg->header.stamp.toSec();
    if (curr_imu_time < last_imu_time) {
        LOG_INFO("Detected imu loop back");
        imu_queue_.clear();
    }
    if (m_node_config.imu_topic == "/livox/imu") {
        rosIMUtoIMU(imu_msg, imu_data, true, false);
    } else {
        rosIMUtoIMU(imu_msg, imu_data, false, false);
    }
    imu_queue_.push_back(imu_data);
    last_imu_time = curr_imu_time;
}
void slam::LIONode::StandardLidarCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
}
void slam::LIONode::LivoxLidarCallback(const livox_ros_driver2::CustomMsg::ConstPtr& livox_msg) {
    PointCloudPtr cloud_ptr = livox2pcl(livox_msg);
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    double current_lidar_time = livox_msg->header.stamp.toSec();
    if (current_lidar_time < last_lidar_time) {
        LOG_ERROR("Detected lidar loop back");
        lidar_queue_.clear();
        lidar_time_queue_.clear();
    }
    lidar_queue_.push_back(cloud_ptr);
    lidar_time_queue_.push_back(current_lidar_time);
    last_lidar_time = current_lidar_time;
}

bool slam::LIONode::sync_package(MeasureGroup& measure) {
    // 队列为空则不处理
    if (lidar_queue_.empty() || imu_queue_.empty() || lidar_time_queue_.empty()) {
        return false;
    }

    // 未处理雷达的情况下
    if (!lidar_pushed_) {
        // 取出第一帧雷达
        measure.curr_cloud = lidar_queue_.front();
        measure.lidar_begin_time = lidar_time_queue_.front();
        // 判断雷达是否有异常
        if (measure.curr_cloud->points.size() <= 1) {
            measure.lidar_end_time = measure.lidar_begin_time + lidar_mean_scantime_;
            LOG_INFO("curr_cloud points size < 1");
        } else if (measure.curr_cloud->points.back().time < 0.5 * lidar_mean_scantime_) {
            measure.lidar_end_time = measure.lidar_begin_time + lidar_mean_scantime_;
        } else {
            scan_num_++;
            measure.lidar_end_time = measure.lidar_begin_time + measure.curr_cloud->points.back().time / 1000.0;
            // LOG_INFO("lidar_begin_time:{},lidar_end_time:{}", measure.lidar_begin_time, measure.lidar_end_time);
            // LOG_INFO("back time:{}", measure.curr_cloud->points.back().time);
            // LOG_INFO("begin sync, lidar_begin_time:{}, lidar_end_time:{}, imu_queue_size:{}",
            //          measure.lidar_begin_time , measure.lidar_end_time , imu_queue_.size());
            lidar_mean_scantime_ += (measure.curr_cloud->points.back().time / 100.0 - lidar_mean_scantime_) / scan_num_;
            LOG_INFO("lidar_mean_scantime:{}", lidar_mean_scantime_);
        }
        lidar_pushed_ = true;
    }
    // 开始同步imu消息
    double imu_time = imu_queue_.front().timestamped_;
    LOG_INFO("imu_time:{}", imu_time);
    measure.imus.clear();
    // 找到第一帧雷达之前的imu
    while (!imu_queue_.empty() && imu_time < measure.lidar_end_time) {
        imu_time = imu_queue_.front().timestamped_;
        if (imu_time > measure.lidar_end_time) {
            break;
        }
        measure.imus.push_back(imu_queue_.front());
        imu_queue_.pop_front();
    }

    // LOG_INFO("find imu size:{} begin lidar:{}", measure.imus.size(), measure.lidar_end_time);
    if (!measure.imus.empty()) {
        LOG_INFO("lidar_begin_time:{},lidar_end_time:{}", measure.lidar_begin_time, measure.lidar_end_time);
        LOG_INFO("imu_begin_time:{}, imu_end_time:{}", measure.imus.begin()->timestamped_,
                 measure.imus.back().timestamped_);
    }
    if (measure.imus.empty()) {
        lidar_queue_.pop_front();
        lidar_time_queue_.pop_front();
        lidar_pushed_ = false;
        return false;
    }

    // 处理gnss
    lidar_queue_.pop_front();
    lidar_time_queue_.pop_front();
    lidar_pushed_ = false;
    return true;
}

void slam::LIONode::run() {
    // 这里频率过快会有问题
    ros::Rate rate(50);
    while (ros::ok()) {
        rate.sleep();
        MeasureGroup measure;
        ros::spinOnce();
        if (!sync_package(measure)) {
            continue;
        }
        LOG_INFO("sync_package success");
        auto odom_start_time = std::chrono::high_resolution_clock::now();
        fastlio_odom_ptr_->ProcessSyncpackage(measure);
        auto odom_end_time = std::chrono::high_resolution_clock::now();
        auto odom_time_used = std::chrono::duration_cast<std::chrono::microseconds>(odom_end_time - odom_start_time);
        LOG_INFO("lio_odom used time:{}", odom_time_used.count() * 1e-6);
        PublishTf(m_node_config.world_frame, m_node_config.body_frame, measure.lidar_end_time);
        PublishOdom(m_node_config.world_frame, m_node_config.body_frame, measure.lidar_end_time);
        PointCloudPtr body_cloud =
            TransformCloud(measure.curr_cloud, ieskf_->GetState().R_LtoI, ieskf_->GetState().t_LinI);
        PublishCloud(body_cloud, m_node_config.body_frame, measure.lidar_end_time);
        PointCloudPtr world_cloud = TransformCloud(measure.curr_cloud, fastlio_odom_ptr_->GetLidarProcess()->GetRLtoG(),
                                                   fastlio_odom_ptr_->GetLidarProcess()->GetTLtoG());
        PublishCloud(body_cloud, m_node_config.body_frame, measure.lidar_end_time);
    }
}

void slam::LIONode::PublishTf(const std::string& parent_frame, const std::string& child_frame, double timestamped) {
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = parent_frame;
    transformStamped.child_frame_id = child_frame;
    transformStamped.header.stamp = ros::Time(timestamped);

    Eigen::Quaterniond q(ieskf_->GetState().R_);
    auto t = ieskf_->GetState().P_;
    transformStamped.transform.rotation.w = q.w();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.translation.z = t.z();
    transformStamped.transform.translation.x = t.x();
    transformStamped.transform.translation.y = t.y();
    tf_broadcaster_.sendTransform(transformStamped);
}

void slam::LIONode::PublishOdom(const std::string& parent_frame, const std::string& child_frame, double timestamped) {
    nav_msgs::Odometry odom;
    odom.header.frame_id = parent_frame;
    odom.child_frame_id = child_frame;
    odom.header.stamp = ros::Time(timestamped);

    odom.pose.pose.position.x = ieskf_->GetState().P_.x();
    odom.pose.pose.position.y = ieskf_->GetState().P_.y();
    odom.pose.pose.position.z = ieskf_->GetState().P_.z();
    Eigen::Quaterniond q(ieskf_->GetState().R_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    Vec3d vel = ieskf_->GetState().R_.transpose() * ieskf_->GetState().V_;
    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = vel.z();
    odom_pub_.publish(odom);
}

void slam::LIONode::PublishCloud(const PointCloudPtr& cloud, const std::string& frame, double timestamped) {
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(*cloud, ros_msg);
    ros_msg.header.frame_id = frame;
    ros_msg.header.stamp = ros::Time(timestamped);
    body_cloud_pub_.publish(ros_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lio_node");
    ros::NodeHandle nh("~");
    std::shared_ptr<slam::LIONode> lio_node_ptr = std::make_shared<slam::LIONode>(nh);

    lio_node_ptr->run();
    // join线程
    LOG_INFO("LioNode Exits");
    return 0;
}