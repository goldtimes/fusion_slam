#pragma onnce

#include <Eigen/Core>
#include <memory>
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

    static std::unique_ptr<SystemConfig>& GetInstance();

   private:
    SystemConfig() = default;

   private:
    static std::unique_ptr<SystemConfig> instance_ptr_;

   public:
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
    // 回环检测的配置类
    struct LoopClosureConfig {};
    // 地图生成的配置类
    struct MappingConfig {};
    // 定位模式后端点云配准的配置类
    struct LocalizedConifg {};
};
}  // namespace slam