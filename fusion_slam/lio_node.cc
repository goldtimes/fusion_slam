#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>
#include <thread>
#include "SystemConfig.hh"
#include "common/eigen_type.hh"
#include "common/logger.hpp"
#include "common_lib.hh"
#include "static_imu_init.hh"

namespace slam {
class LIONode {
   public:
    struct LIONodeConfig {
        std::string imu_topic;
        std::string lidar_topic;
        bool livox_msg;
        std::string body_frame;
        std::string world_frame;
        bool print_time_cost;

        int lidar_filter_num = 3;
        double lidar_min_range = 0.5;
        double lidar_max_range = 20.0;
        double scan_resolution = 0.15;
        double map_resolution = 0.3;

        double cube_len = 300;
        double det_range = 60;
        double move_thresh = 1.5;

        double na = 0.01;
        double ng = 0.01;
        double nba = 0.0001;
        double nbg = 0.0001;
        int imu_init_num = 20;
        int near_search_num = 5;
        int ieskf_max_iter = 5;
        bool gravity_align = true;
        bool esti_il = false;
        Mat3d r_il = Mat3d::Identity();
        Vec3d t_il = Vec3d::Zero();

        double lidar_cov_inv = 1000.0;
    };

    LIONode(const ros::NodeHandle nh);
    ~LIONode() = default;

    void run();

   private:
    void loadParames();

    void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void StandardLidarCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    void LivoxLidarCallback(const livox_ros_driver::CustomMsg::ConstPtr& livox_msg);

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
};
}  // namespace slam

slam::LIONode::LIONode(const ros::NodeHandle nh) : nh_(nh) {
    loadParames();
    imu_sub_ = nh_.subscribe(m_node_config.imu_topic, 1000, &LIONode::ImuCallback, this);
    lidar_sub_ = m_node_config.livox_msg
                     ? nh_.subscribe(m_node_config.lidar_topic, 10, &LIONode::LivoxLidarCallback, this)
                     : nh_.subscribe(m_node_config.lidar_topic, 10, &LIONode::StandardLidarCallback, this);
    body_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("body_cloud", 10);
    world_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("world_cloud", 10);
    path_pub_ = nh_.advertise<nav_msgs::Path>("lio_path", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("lio_odom", 10);
    sync_thread_ = std::thread(&LIONode::run, this);
}

void slam::LIONode::loadParames() {
    std::string config_path;
    nh_.param<std::string>("config_path", config_path);

    YAML::Node config = YAML::LoadFile(config_path);
    if (!config) {
        LOG_INFO("FAIL TO LOAD YAML FILE!");
        return;
    }

    LOG_INFO("LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "lio_node");
    ros::NodeHandle nh;
    std::shared_ptr<slam::LIONode> lio_node_ptr = std::make_shared<slam::LIONode>(nh);

    lio_node_ptr->run();
    ros::spin();
    // join线程
    LOG_INFO("LioNode Exits");
    return 0;
}