/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-14 02:40:29
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-14 03:00:46
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/SystemConfig.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "SystemConfig.hh"
#include <mutex>

namespace slam {
// 静态成员变量初始化没什么私有不私有的问题
std::unique_ptr<SystemConfig> SystemConfig::instance_ptr_ = nullptr;

std::unique_ptr<SystemConfig>& SystemConfig::GetInstance() {
    if (instance_ptr_ == nullptr) {
        // 用call_once来做单例
        static std::once_flag flag;
        std::call_once(flag, [&]() { instance_ptr_.reset(new SystemConfig()); });
    }
    return instance_ptr_;
}
}  // namespace slam