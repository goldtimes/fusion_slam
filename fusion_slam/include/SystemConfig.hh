#pragma onnce

#include <Eigen/Core>
#include <boost/filesystem.hpp>
#include <memory>
#include <string>
#include "common/PoseTrans.hpp"
#include "common/logger.hpp"
#include "sensors/lidar.hh"
/**
    系统的配置类,用单例的方式来实现
*/
namespace slam {
class SystemConfig {
   public:
    // 拷贝构造
    SystemConfig(const SystemConfig& other) = delete;
    // 赋值构造
    SystemConfig& operator=(const SystemConfig& other) = delete;
    // 移动构造函数
    SystemConfig(SystemConfig&& other) = delete;
    // 这里最后不要返回unique_ptr，uniqur无法拷贝
    static SystemConfig& GetInstance();

    void SetConfigPath(const std::string& path);

   private:
    SystemConfig() = default;

   private:
    static std::unique_ptr<SystemConfig> instance_ptr_;

   public:
    std::string config_path;
    // lidar->imu的外参信息
    PoseTranse<double> T_I_L;
    // lidar->轮速计的外参信息
    PoseTranse<double> T_E_L;
    std::string lidar_topic;
    std::string imu_topic;
    std::string encorder_topic;
    std::string gnss_topic;

    enum class FUSION_MODE {
        // 滤波的方案
        IESKF_MODE = 1,
        // 优化的方案
        OPTIMIZE = 2
    };
    // 前端里程计的配置类
    struct FrontEndConfig {
        // 是否为9轴的imu
        bool axis9_imu{false};
        // 默认使用ieskf
        FUSION_MODE fusin_mode{FUSION_MODE::IESKF_MODE};
    };

    FrontEndConfig frontend_config;

    // 雷达的配置类
    struct LidarConfig {
        std::string lidar_type_ = "";
        // 雷达线束
        int lidar_scan{};
        int lidar_horizon_scan{};
        // 过滤
        int lidar_point_filter{};
        // 垂直分辨率
        double lidar_vertical_resolution{};
        double lidar_lower_angle{};
        // 雷达的时间scale
        double lidar_time_scale{};
        double lidar_rotation_noise{};
        double lidar_position_noise{};
        double lidar_min_dist{};
        double lidar_max_dist{};
    };
    LidarConfig lidar_config;

    // imu初始化的配置类
    struct StaticImuInitConfig {
        // 静止的时间
        double init_time{2.0};
        // 是否使用轮速计
        bool use_odom{false};
        // 初始化的imu最大size
        int max_init_size{200};
        // 初始化acc和gyro的norm阈值
        double max_static_gyro_var{0.5};
        double max_static_acc_var{0.05};
        double gravity_norm_{9.81};
    };
    StaticImuInitConfig imu_init_config;
    // 回环检测的配置类
    struct LoopClosureConfig {};
    // 地图生成的配置类
    struct MappingConfig {};
    // 定位模式后端点云配准的配置类
    struct LocalizedConfig {};
};
}  // namespace slam