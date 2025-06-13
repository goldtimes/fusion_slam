/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-14 02:14:47
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-14 02:35:33
 * @FilePath: /fusion_slam_ws/src/fusion_slam/app/fusion_slam_node.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <ros/ros.h>
#include <ros/rate.h>
#include "common/logger.hpp"

int main(int argc, char ** argv){
    ros::init(argc, argv, "fusion_slam_node");
    ros::NodeHandle nh_;
    ros::Rate rate(1000);
    LOG_INFO("[Fusion Slam Node Start!]");
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    LOG_INFO("[Fusion Slam Node Exit!]");
    return 0;
}