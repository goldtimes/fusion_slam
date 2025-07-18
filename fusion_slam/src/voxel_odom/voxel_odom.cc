/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-07-15 23:13:59
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-07-17 00:48:20
 * @FilePath: /fusion_slam_ws/src/fusion_slam/src/voxel_odom/voxel_odom.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "voxel_odom/voxel_odom.hh"
#include <pcl/pcl_macros.h>
#include <cmath>
#include <vector>
#include "common/common_lib.hh"
#include "sensors/lidar.hh"

namespace slam {
VoxelOdom::VoxelOdom(const VoxelOdomConfig& config) {
    LOG_INFO("Voxel Odom Init");
    // 初始化ESIKF
    kf_ = std::make_shared<esekfom::esekf<state_ikfom, PROCESS_NOISE_DOF, input_ikfom>>();
    std::vector<double> epsi(23, 0.001);
    kf_->init_dyn_share(
        get_f, df_dx, df_dw,
        [this](state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) { sharedUpdateFunc(s, ekfom_data); },
        params.esikf_max_iteration, epsi.data());

    // 初始化IMUProcessor
    IMUProcessor::IMUProcessorConfig imu_process_config_;
    imu_process_config_.align_gravity = config.align_gravity;
    imu_processor_ptr_ = std::make_shared<IMUProcessor>(imu_process_config_, kf_);
    // 设置外参
    imu_processor_ptr_->setExtParams(config.imu_ext_rot, config.imu_ext_pos);
    // 设置Q_过程噪声
    imu_processor_ptr_->SetCov(params.imu_gyro_cov, params.imu_acc_cov, params.imu_gyro_bias_cov,
                               params.imu_acc_bias_cov);
    voxel_map_ = std::make_shared<VoxelMap>(config.voxel_size, config.max_layer, config.update_size_threshes,
                                            config.max_point_thresh, config.plane_thresh);
    // 初始化点云采样器
    voxel_filter_.setLeafSize(params.scan_resolution, params.scan_resolution, params.scan_resolution);

    extrinsic_est_en_ = params.extrinsic_est_en;
    // 全量去畸变都的点云
    cloud_undistorted_lidar_.reset(new PointCloud);
    // 降采样后的点云
    cloud_down_lidar_.reset(new PointCloud);
    cloud_down_world_.reset(new PointCloud);
    // feats_with_correspondence.reset(new PointCloud);
    residual_info.resize(10000);
}

void VoxelOdom::mapping(MeasureGroup& sync_packag) {
    // imu 初始化
    if (!imu_processor_ptr_->GetInitSuccess()) {
        imu_processor_ptr_->TryInit(sync_packag);
        return;
    }
    // imu传播和去畸变
    cloud_undistorted_lidar_.reset(new PointCloud);
    imu_processor_ptr_->PredictAndUndistort(sync_packag, cloud_undistorted_lidar_);
    // 滤波
    voxel_filter_.setInputCloud(cloud_undistorted_lidar_);
    voxel_filter_.filter(*cloud_down_lidar_);
    // 清空该点云
    cloud_undistorted_lidar_.reset(new PointCloud);
    if (system_status_ == SYSTEM_STATUES::INITIALIZE) {
        // 转换点云到世界坐标系下
        cloud_down_world_ = transformWorld(cloud_down_lidar_);
        std::vector<PointWithCov> pv_list;
        for (int i = 0; i < cloud_down_world_->size(); ++i) {
            PointWithCov pv;
            V3D point_lidar(cloud_down_lidar_->points[i].x, cloud_down_lidar_->points[i].y,
                            cloud_down_lidar_->points[i].z);
            V3D point_world(cloud_down_world_->points[i].x, cloud_down_world_->points[i].y,
                            cloud_down_world_->points[i].z);
            // 防止为0
            if (point_lidar[2] == 0.0) {
                point_lidar[2] = 0.001;
            }
            M3D cov_lidar = calcBodyCov(point_lidar, params.ranging_cov, params.angle_cov);
            // 计算世界坐标下的cov
            M3D cov_world = transformLidarCovToWorld(point_lidar, cov_lidar);
            pv.point = point_world;
            pv.cov = cov_world;
            pv_list.push_back(pv);
        }
        // 构建第一帧的voxel_map
        // buildVoxelMap(pv_list, params.voxel_size, params.max_layer, params.update_size_threshes,
        //               params.max_point_thresh, params.max_point_cov_thresh, params.plane_thresh, voxel_map_);
        voxel_map_->insert(pv_list);
        LOG_INFO("Build Voxel Map Size:{}", pv_list.size());
        system_status_ = SYSTEM_STATUES::MAPPING;
        return;
    }
    // align
    std::sort(cloud_down_lidar_->begin(), cloud_down_lidar_->end(),
              [](const PointType& p1, const PointType& p2) { return p1.curvature < p2.curvature; });
    auto point_size = cloud_down_lidar_->size();
    if (point_size < 5) {
        LOG_ERROR("No point, skip this scan");
    }
    // 计算lidar每个点的协方差
    vars_cloud_.clear();
    vars_cloud_.reserve(point_size);
    int i = 0;
    for (auto& pt : cloud_down_lidar_->points) {
        V3D point_lidar(pt.x, pt.y, pt.z);
        residual_info[i].point_lidar = point_lidar;
        M3D point_cov = calcBodyCov(point_lidar, params.ranging_cov, params.angle_cov);
        vars_cloud_.push_back(point_cov);
        i++;
    }
    double solve_H_time;
    auto start = std::chrono::high_resolution_clock::now();
    kf_->update_iterated_dyn_share_modified(0.001, solve_H_time);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOG_INFO("Align By Voxel used time:{}", duration.count() * 1e-3);
    // print state
    // Update Voxel map
    std::vector<PointWithCov> pv_world_list;
    pv_world_list.reserve(cloud_down_lidar_->size());
    // 根据最新的状态，将点云转到world系
    PointCloudPtr cloud_world = transformWorld(cloud_down_lidar_);
    for (size_t i = 0; i < cloud_world->size(); ++i) {
        PointWithCov pv;
        V3D point_lidar(cloud_down_lidar_->points[i].x, cloud_down_lidar_->points[i].y, cloud_down_lidar_->points[i].z);
        V3D point_wolrd(cloud_world->points[i].x, cloud_world->points[i].y, cloud_world->points[i].z);

        M3D point_lidar_cov = vars_cloud_[i];
        M3D point_world_cov = transformLidarCovToWorld(point_lidar, point_lidar_cov);
        pv.point = point_wolrd;
        pv.cov = point_world_cov;
        pv_world_list.push_back(pv);
    }
    start = std::chrono::high_resolution_clock::now();
    // std::sort(pv_world_list.begin(), pv_world_list.end(), [](const PointWithCov& pv_1, const PointWithCov& pv_2) {
    //     return pv_1.cov.diagonal().norm() < pv_2.cov.diagonal().norm();
    // });
    // updateVoxelMapOMP(pv_world_list, params.voxel_size, params.max_layer, params.update_size_threshes,
    //                   params.max_point_thresh, params.max_point_cov_thresh, params.plane_thresh, voxel_map_);
    voxel_map_->insert(pv_world_list);
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOG_INFO("Update Voxel Map", duration.count() * 1e-3);
    // 最后保存世界坐标系下的点云
    cloud_down_world_ = transformWorld(cloud_down_lidar_);
}

M3D VoxelOdom::calcBodyCov(const V3D& point_lidar, double range_inc, double angle_inc) {
    // 距离
    double range =
        sqrt(point_lidar[0] * point_lidar[0] + point_lidar[1] * point_lidar[1] + point_lidar[2] * point_lidar[2]);
    // 距离噪声
    double range_var = range_inc * range_inc;
    // 方向噪声
    Eigen::Matrix2d direction_var;
    direction_var << pow(sin(DEG2RAD(angle_inc)), 2), 0, 0, pow(sin(DEG2RAD(angle_inc)), 2);
    V3D direction(point_lidar);
    // 归一化
    direction.normalize();
    M3D direction_hat;
    // clang-format off
    direction_hat << 0, -direction(2), direction(1),
                    direction(2), 0, -direction(0),
                    -direction(1), direction(2), 0;
    // clang-format on
    V3D base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();

    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();
    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
    return direction * range_var * direction.transpose() + A * direction_var * A.transpose();
}

M3D VoxelOdom::transformLidarCovToWorld(const V3D& point_lidar, const M3D& cov_lidar) {
    M3D point_hat;
    point_hat << SKEW_SYM_MATRX(point_lidar);
    auto state = kf_->get_x();
    M3D R_IL_cov = kf_->get_P().block<3, 3>(6, 6);
    M3D t_IL_cov = kf_->get_P().block<3, 3>(9, 9);
    M3D cov_body = state.offset_R_L_I * cov_lidar * state.offset_R_L_I.conjugate() +
                   state.offset_R_L_I * (-point_hat) * R_IL_cov * (-point_hat).transpose() + t_IL_cov;
    // 转到到body坐标系
    V3D point_body = state.offset_R_L_I * point_lidar + state.offset_T_L_I;

    // 计算世界坐标系下的cov;
    point_hat << SKEW_SYM_MATRX(point_body);
    M3D rot_var = kf_->get_P().block<3, 3>(3, 3);
    M3D trans_var = kf_->get_P().block<3, 3>(0, 0);
    M3D cov_world = state.rot * cov_body * state.rot.conjugate() +
                    state.rot * (-point_hat) * rot_var * (-point_hat).transpose() * state.rot.conjugate() + trans_var;
    return cov_world;
}
void VoxelOdom::sharedUpdateFunc(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data) {
    M3D r_wl = state.rot.matrix() * state.offset_R_L_I;
    V3D p_wl = state.rot.matrix() * state.offset_T_L_I + state.pos;

    int size = cloud_down_lidar_->size();
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < size; i++) {
        residual_info[i].is_valid = false;
        residual_info[i].current_layer = 0;
        residual_info[i].from_near = false;
        residual_info[i].point_world = r_wl * residual_info[i].point_lidar + p_wl;

        Eigen::Matrix3d point_crossmat = Sophus::SO3d::hat(residual_info[i].point_lidar);
        residual_info[i].cov = r_wl * residual_info[i].pcov * r_wl.transpose() +
                               point_crossmat * kf_->get_P().block<3, 3>(3, 3) * point_crossmat.transpose() +
                               kf_->get_P().block<3, 3>(0, 0);
        VOXEL_KEY position = voxel_map_->calc_index(residual_info[i].point_world);
        auto iter = voxel_map_->feat_map.find(position);
        if (iter != voxel_map_->feat_map.end()) {
            voxel_map_->buildResidual(residual_info[i], iter->second.tree);
        }
    }
    ekfom_data.h_x.setZero();
    ekfom_data.h.setZero();
    Eigen::Matrix<double, 1, 12> J;
    Eigen::Matrix<double, 1, 6> J_v;
    int effect_num = 0;

    for (int i = 0; i < size; i++) {
        if (!residual_info[i].is_valid) continue;
        effect_num++;
        J.setZero();
        J_v.block<1, 3>(0, 0) = (residual_info[i].point_world - residual_info[i].plane_center).transpose();
        J_v.block<1, 3>(0, 3) = -residual_info[i].plane_norm.transpose();
        double r_cov = J_v * residual_info[i].plane_cov * J_v.transpose();
        r_cov += residual_info[i].plane_norm.transpose() * r_wl * residual_info[i].pcov * r_wl.transpose() *
                 residual_info[i].plane_norm;
        double r_info = r_cov < 0.0001 ? 1000 : 1 / r_cov;
        assert(r_cov > 0.0);
        J.block<1, 3>(0, 0) = residual_info[i].plane_norm.transpose();
        J.block<1, 3>(0, 3) = -residual_info[i].plane_norm.transpose() * state.rot *
                              Sophus::SO3d::hat(state.offset_R_L_I * residual_info[i].point_lidar + state.offset_T_L_I);
        if (params.extrinsic_est_en) {
            J.block<1, 3>(0, 6) =
                -residual_info[i].plane_norm.transpose() * r_wl * Sophus::SO3d::hat(residual_info[i].point_lidar);
            J.block<1, 3>(0, 9) = residual_info[i].plane_norm.transpose() * state.rot;
        }
        ekfom_data.h_x += J.transpose() * r_info * J;
        ekfom_data.h += J.transpose() * r_info * residual_info[i].residual;
    }
    if (effect_num < 1) std::cout << "NO EFFECTIVE POINT";
}

// void VoxelOdom::sharedUpdateFunc(state_ikfom& state, esekfom::dyn_share_datastruct<double>& ekfom_data) {
//     // feats_with_correspondence->clear();
//     total_residual = 0.0;
//     // 迭代卡尔曼滤波，根据最新的位姿来讲点云转到世界坐标系
//     std::vector<PointWithCov> pv_list;
//     PointCloudPtr cloud_world(new PointCloud);
//     cloud_world = transformWorld(cloud_down_lidar_);
//     LOG_INFO("[Voxel Align] cloud size:{}", cloud_world->size());
//     pv_list.resize(cloud_world->size());
//     for (size_t i = 0; i < cloud_down_lidar_->size(); ++i) {
//         PointWithCov pv;
//         V3D point_lidar(cloud_down_lidar_->points[i].x, cloud_down_lidar_->points[i].y,
//         cloud_down_lidar_->points[i].z); V3D point_world(cloud_world->points[i].x, cloud_world->points[i].y,
//         cloud_world->points[i].z); M3D point_lidar_var = vars_cloud_[i]; M3D point_world_var =
//         transformLidarCovToWorld(point_lidar, point_lidar_var); pv.point = point_lidar;
//         // pv.point_world = point_world;
//         // LOG_INFO("[i]:{},{},{}", point_lidar.x(), point_lidar.y(), point_lidar.z());

//         // pv.cov_lidar = point_lidar_var;
//         pv.cov = point_world_var;
//         pv_list[i] = pv;
//     }
//     // 构建残差
//     auto match_start = std::chrono::high_resolution_clock::now();
//     std::vector<ptpl> ptpl_list;
//     std::vector<V3D> non_match_list;
//     BuildResidualListOMP(voxel_map_, params.voxel_size, 3.0, params.max_layer, pv_list, ptpl_list, non_match_list);
//     auto match_end = std::chrono::high_resolution_clock::now();
//     // 接下里就是填充观测矩阵
//     auto effct_feat_num = ptpl_list.size();
//     if (effct_feat_num < 5) {
//         ekfom_data.valid = false;
//         LOG_INFO("No Effective Points");
//         return;
//     }
//     LOG_INFO("find effective Points:{}", effct_feat_num);
//     // 找到配准的点数，以及对位姿和外参的更新12维
//     ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, 12);
//     // 残差
//     ekfom_data.h.resize(effct_feat_num);
//     ekfom_data.R.resize(effct_feat_num, 1);
// #ifdef MP_EN
//     omp_set_num_threads(MP_PROC_NUM);
// #pragma omp parallel for
// #endif
//     for (int i = 0; i < effct_feat_num; ++i) {
//         V3D point_lidar = ptpl_list[i].point;
//         M3D point_lidar_hat;
//         point_lidar_hat << SKEW_SYM_MATRX(point_lidar);
//         V3D point_lidar_body;
//         point_lidar_body = state.offset_R_L_I * point_lidar + state.offset_T_L_I;
//         M3D point_body_hat;
//         point_body_hat << SKEW_SYM_MATRX(point_lidar_body);
//         V3D normal_vec(ptpl_list[i].normal);
//         // 雅可比矩阵
//         V3D C(state.rot.conjugate() * normal_vec);
//         V3D A(point_lidar_hat * C);
//         if (params.extrinsic_est_en) {
//             V3D B(point_lidar_hat * state.offset_R_L_I.conjugate() * C);
//             ekfom_data.h_x.block<1, 12>(i, 0) << normal_vec.x(), normal_vec.y(), normal_vec.z(), VEC_FROM_ARRAY(A),
//                 VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
//         } else {
//             ekfom_data.h_x.block<1, 12>(i, 0) << normal_vec.x(), normal_vec.y(), normal_vec.z(), VEC_FROM_ARRAY(A),
//             0,
//                 0, 0, 0, 0, 0;
//         }
//         // 点到平面的距离
//         float pd2 = normal_vec.x() * ptpl_list[i].point_world.x() + normal_vec.y() * ptpl_list[i].point_world.y() +
//                     normal_vec.z() * ptpl_list[i].point_world.z() + ptpl_list[i].d;
//         ekfom_data.h(i) = -pd2;
//         // LOG_INFO("norm[i]:{},{},{}", normal_vec.x(), normal_vec.y(), normal_vec.z());
//         // LOG_INFO("point[i]:{},{},{}", ptpl_list[i].point_world.x(), ptpl_list[i].point_world.y(),
//         //          ptpl_list[i].point_world.z());

//         total_residual += pd2;
//         V3D point_world = ptpl_list[i].point_world;
//         // /*** get the normal vector of closest surface/corner ***/
//         Eigen::Matrix<double, 1, 6> J_nq;
//         J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[i].center;
//         J_nq.block<1, 3>(0, 3) = -ptpl_list[i].normal;
//         double sigma_l = J_nq * ptpl_list[i].plane_cov * J_nq.transpose();

//         // M3D cov_lidar = calcBodyCov(ptpl_list[i].point, ranging_cov, angle_cov);
//         M3D cov_lidar = ptpl_list[i].cov_lidar;
//         M3D R_cov_Rt =
//             state.rot * state.offset_R_L_I * cov_lidar * state.offset_R_L_I.conjugate() * state.rot.conjugate();
//         // HACK 1. 因为是标量 所以求逆直接用1除
//         // HACK 2. 不同分量的方差用加法来合成 因为公式(12)中的Sigma是对角阵，逐元素运算之后就是对角线上的项目相加
//         double R_inv = 1.0 / (sigma_l + normal_vec.transpose() * R_cov_Rt * normal_vec);

//         // 计算测量方差R并赋值 目前暂时使用固定值
//         // ekfom_data.R(i) = 1.0 / LASER_POINT_COV;
//         ekfom_data.R(i) = R_inv;
//     }

//     res_mean_last = total_residual / effct_feat_num;
//     LOG_INFO("total_residual:{}, res_mean_last:{}", total_residual, res_mean_last);
// }

PointCloudPtr VoxelOdom::transformWorld(const PointCloudPtr& cloud_in) {
    PointCloudPtr cloud_world(new PointCloud);
    if (cloud_in->empty()) {
        return cloud_world;
    }
    cloud_world->reserve(cloud_in->size());
    // 当前的状态
    M3D R_wi = kf_->get_x().rot.matrix();
    V3D p_wi = kf_->get_x().pos;
    M3D r_il = kf_->get_x().offset_R_L_I.matrix();
    V3D t_il = kf_->get_x().offset_T_L_I;
    for (int i = 0; i < cloud_in->size(); ++i) {
        auto point_lidar_eigen = PointToEigen(cloud_in->points[i]);
        auto point_world_eigen = R_wi * (r_il * point_lidar_eigen + t_il) + p_wi;
        auto point_world = EigenToPoint(point_world_eigen);
        point_world.intensity = cloud_in->points[i].intensity;
        point_world.curvature = cloud_in->points[i].curvature;
        cloud_world->push_back(point_world);
    }
    return cloud_world;
}

PointCloudPtr VoxelOdom::cloudUndistortedBody() {
    PointCloudPtr cloud_undistorted_body(new PointCloud);
    Eigen::Matrix3d rot = kf_->get_x().offset_R_L_I.toRotationMatrix();
    Eigen::Vector3d pos = kf_->get_x().offset_T_L_I;
    cloud_undistorted_body->reserve(cloud_down_lidar_->size());
    for (auto& p : cloud_down_lidar_->points) {
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = rot * point + pos;
        PointType p_body;
        p_body.x = point(0);
        p_body.y = point(1);
        p_body.z = point(2);
        p_body.intensity = p.intensity;
        cloud_undistorted_body->points.push_back(p_body);
    }
    return cloud_undistorted_body;
}

}  // namespace slam