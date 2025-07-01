/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-02 00:10:49
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-02 00:14:36
 * @FilePath: /fusion_slam_ws/src/fusion_slam/include/fastlio_odom/fastkio_ieskf.hh
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <Eigen/Core>
#include <sophus/so3.hpp>

namespace slam::fastlio {

using M12D = Eigen::Matrix<double, 12, 12>;
// 21为r,t,r_li,t_li, v, bg, ba的状态量
using M21D = Eigen::Matrix<double, 21, 21>;

using V12D = Eigen::Matrix<double, 12, 1>;
using V21D = Eigen::Matrix<double, 21, 1>;

class IESKF {
   public:
   private:
};
}  // namespace slam::fastlio