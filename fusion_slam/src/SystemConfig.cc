/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-14 02:40:29
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-14 08:02:02
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/SystemConfig.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "SystemConfig.hh"
#include <boost/filesystem/path.hpp>
#include <cstdlib>
#include <mutex>
#include "common/logger.hpp"

namespace slam {
// 静态成员变量初始化没什么私有不私有的问题
std::unique_ptr<SystemConfig> SystemConfig::instance_ptr_ = nullptr;

SystemConfig& SystemConfig::GetInstance() {
    if (instance_ptr_ == nullptr) {
        // 用call_once来做单例
        static std::once_flag flag;
        std::call_once(flag, [&]() { instance_ptr_.reset(new SystemConfig()); });
    }
    return *instance_ptr_;
}

void SystemConfig::SetConfigPath(const std::string& path) {
    if (boost::filesystem::exists(boost::filesystem::path(path))) {
        LOG_INFO("Load config path:{}", path);
        config_path = path;
    } else {
        LOG_ERROR("Config path no exists!");
        exit(0);
    }
}
}  // namespace slam