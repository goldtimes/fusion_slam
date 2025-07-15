#include "voxel_odom/voxel_map.hh"
#include <sys/types.h>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "common/logger.hh"

namespace slam {

void OctoTree::init_octo_tree() {
    //  > 5个点，开始拟合平面
    if (temp_points_.size() > max_plane_update_threshold_) {
        // 主成分分析
        init_plane(temp_points_, plane_ptr_);
        if (plane_ptr_->is_plane) {
            // 如果是平面
            octo_state_ = 0;  // end of tree,如果是平面就不切割了，标记为叶子节点
            if (temp_points_.size() > max_cov_points_size_) {
                update_cov_enable_ = false;
            }
            if (temp_points_.size() > max_points_size_) {
                update_enable_ = false;
            }
        } else {
            // 不是平面
            octo_state_ = 1;
            // 往下切割体素
            cut_octor_tree();
        }
        // 设置标记
        init_octo_ = true;
        new_points_num_ = 0;
    }
}

/**
    PCA
    能够找到数据变化最小的方向，而平面拟合的目标正是找到一个超平面，使得所有点到该平面的距离平方和最小。这两者是等价的：
    最小特征值对应的特征向量方向上的数据变化最小
    该方向垂直于数据分布的平面
    因此，这个特征向量就是平面的法向量
*/
void OctoTree::init_plane(const std::vector<PointWithCov>& input_points, Plane* plane) {
    // 平面协方差
    plane->plane_cov = Mat6d::Zero();
    plane->covariance = M3D::Zero();
    plane->center = V3D::Zero();
    plane->normal = V3D::Zero();
    plane->points_size = input_points.size();
    plane->radius = 0;
    // 统计均值和协方差
    for (auto pv : input_points) {
        plane->center += pv.point;
        plane->covariance += pv.point * pv.point.transpose();
    }
    plane->center = plane->center / plane->points_size;
    plane->covariance = plane->covariance / plane->points_size - plane->center * plane->center.transpose();

    // 特征值分解
    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
    // 特征向量
    Eigen::Matrix3cd evecs = es.eigenvectors();
    // 特征值
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();
    // 特征值排序
    Eigen::Matrix3f::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;
    // 特征向量
    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);

    // plane covariance calculation
    // 这里就引入的点的不确定，来估计平面的协方差
    // 雅可比的矩阵就是法向量对p和法向量对n的求导
    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size;
    if (evalsReal(evalsMin) < planer_threshold_) {
        std::vector<int> index(input_points.size());
        std::vector<Eigen::Matrix<double, 6, 6>> temp_matrix(input_points.size());
        for (int i = 0; i < input_points.size(); i++) {
            Eigen::Matrix<double, 6, 3> J;
            Eigen::Matrix3d F;
            for (int m = 0; m < 3; m++) {
                if (m != (int)evalsMin) {
                    Eigen::Matrix<double, 1, 3> F_m = (input_points[i].point - plane->center).transpose() /
                                                      ((plane->points_size) * (evalsReal[evalsMin] - evalsReal[m])) *
                                                      (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                                                       evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
                    F.row(m) = F_m;
                } else {
                    Eigen::Matrix<double, 1, 3> F_m;
                    F_m << 0, 0, 0;
                    F.row(m) = F_m;
                }
            }
            J.block<3, 3>(0, 0) = evecs.real() * F;
            J.block<3, 3>(3, 0) = J_Q;
            plane->plane_cov += J * input_points[i].cov * J.transpose();
        }
    }
    // 平面法向量
    plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
    // y轴的特征值
    plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
    // x轴的特征值
    plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
    plane->mid_eigen_value = evalsReal(evalsMin);
    plane->mid_eigen_value = evalsReal(evalsMid);
    plane->max_eigen_value = evalsReal(evalsMax);

    plane->radius = sqrt(evalsReal(evalsMax));
    plane->d = -(plane->normal(0) * plane->center(0) + plane->normal(1) * plane->center(1) +
                 plane->normal(2) * plane->center(2));
}

void OctoTree::update_plane(const std::vector<PointWithCov>& points, Plane* plane) {
    Eigen::Matrix3d old_covariance = plane->covariance;
    Eigen::Vector3d old_center = plane->center;
    Eigen::Matrix3d sum_ppt = (plane->covariance + plane->center * plane->center.transpose()) * plane->points_size;
    Eigen::Vector3d sum_p = plane->center * plane->points_size;
    for (size_t i = 0; i < points.size(); i++) {
        Eigen::Vector3d pv = points[i].point;
        sum_ppt += pv * pv.transpose();
        sum_p += pv;
    }
    plane->points_size = plane->points_size + points.size();
    plane->center = sum_p / plane->points_size;
    plane->covariance = sum_ppt / plane->points_size - plane->center * plane->center.transpose();
    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();
    Eigen::Matrix3d::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;
    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);
    if (evalsReal(evalsMin) < planer_threshold_) {
        plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
        plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
        plane->min_eigen_value = evalsReal(evalsMin);
        plane->mid_eigen_value = evalsReal(evalsMid);
        plane->max_eigen_value = evalsReal(evalsMax);
        plane->radius = sqrt(evalsReal(evalsMax));
        plane->d = -(plane->normal(0) * plane->center(0) + plane->normal(1) * plane->center(1) +
                     plane->normal(2) * plane->center(2));

        plane->is_plane = true;
        plane->is_update = true;
    } else {
        plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
        plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
        plane->min_eigen_value = evalsReal(evalsMin);
        plane->mid_eigen_value = evalsReal(evalsMid);
        plane->max_eigen_value = evalsReal(evalsMax);
        plane->radius = sqrt(evalsReal(evalsMax));
        plane->d = -(plane->normal(0) * plane->center(0) + plane->normal(1) * plane->center(1) +
                     plane->normal(2) * plane->center(2));
        plane->is_plane = false;
        plane->is_update = true;
    }
}

// 切割体素
void OctoTree::cut_octor_tree() {
    // 如果当前的layer > max_layer，则不切割
    if (layer_ >= max_layer_) {
        octo_state_ = 0;
        return;
    }
    // 对temp_points中的点进行计算，判断在哪些子体素中
    for (size_t i = 0; i < temp_points_.size(); ++i) {
        int xyz[3] = {0, 0, 0};
        // 000, 001, 010,100, 011,,101, 110, 111
        if (temp_points_[i].point.x() > voxel_center_[0]) {
            xyz[0] = 1;
        }
        if (temp_points_[i].point.y() > voxel_center_[1]) {
            xyz[1] = 1;
        }
        if (temp_points_[i].point.z() > voxel_center_[2]) {
            xyz[2] = 1;
        }
        // 计算索引
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        // 判读子节点是否为nullptr
        if (leaves_[leafnum] == nullptr) {
            leaves_[leafnum] = new OctoTree(max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
                                            max_cov_points_size_, planer_threshold_);
            // 计算子体素的中心
            leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
        }
        leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
        leaves_[leafnum]->new_points_num_++;
    }
    // 遍历子节点，初始化平面
    for (uint i = 0; i < 8; ++i) {
        // 不为空的节点
        if (leaves_[i] != nullptr) {
            if (leaves_[i]->temp_points_.size() > max_plane_update_threshold_) {
                // 拟合平面
                init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
                if (plane_ptr_->is_plane) {
                    leaves_[i]->octo_state_ = 0;
                } else {
                    leaves_[i]->octo_state_ = 1;
                    leaves_[i]->cut_octor_tree();
                }
                leaves_[i]->init_octo_ = true;
                leaves_[i]->new_points_num_ = 0;
            }
        }
    }
}
/**
    更新八叉树节点
 */
void OctoTree::UpdateOctoTree(const PointWithCov& pv) {
    if (!init_octo_) {
        // 创建了该子节点，但是点数不够max_plane_update_threshold_
        new_points_num_++;
        all_points_num_++;
        temp_points_.push_back(pv);
        if (temp_points_.size() > max_plane_update_threshold_) {
            init_octo_tree();
        }
    }
    // 已经估计过了平面，也就是 点数 > max_plane_update_threshold_
    else {
        if (plane_ptr_->is_plane) {
            // 平面拟合成功了
            if (update_enable_) {
                // 点数没达到 max_points_size_;
                new_points_num_++;
                all_points_num_++;
                if (update_cov_enable_) {
                    // 更新协方差
                    temp_points_.push_back(pv);
                } else {
                    new_points_.push_back(pv);
                }
                if (new_points_.size() > update_size_threshold_) {
                    if (update_cov_enable_) {
                        // 这里不成立了吧
                        init_plane(new_points_, plane_ptr_);
                    }
                    new_points_num_ = 0;
                }
                if (all_points_num_ >= max_points_size_) {
                    update_cov_enable_ = false;
                    // 清空temp points
                    std::vector<PointWithCov>().swap(temp_points_);
                }
                if (all_points_num_ >= max_cov_points_size_) {
                    update_enable_ = false;
                    plane_ptr_->update_enable = false;
                    // 清空temp points
                    std::vector<PointWithCov>().swap(temp_points_);
                }
            } else {
                return;
            }
        } else {
            // 平面拟合失败,说明体素中的点比较散，我们要到子节点中更新
            if (layer_ < max_layer_) {
                // 当前layer未到达最大层的时候，需要重新计算点在哪个体素中
                if (temp_points_.size() != 0) {
                    std::vector<PointWithCov>().swap(temp_points_);
                }
                if (new_points_.size() != 0) {
                    std::vector<PointWithCov>().swap(new_points_);
                }
                int xyz[3] = {0, 0, 0};
                if (pv.point[0] > voxel_center_[0]) {
                    xyz[0] = 1;
                }
                if (pv.point[1] > voxel_center_[1]) {
                    xyz[1] = 1;
                }
                if (pv.point[2] > voxel_center_[2]) {
                    xyz[2] = 1;
                }
                int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
                if (leaves_[leafnum] != nullptr) {
                    leaves_[leafnum]->UpdateOctoTree(pv);
                } else {
                    leaves_[leafnum] = new OctoTree(max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
                                                    max_cov_points_size_, planer_threshold_);
                    leaves_[leafnum]->layer_point_size_ = layer_point_size_;
                    leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                    leaves_[leafnum]->quater_length_ = quater_length_ / 2;
                    leaves_[leafnum]->UpdateOctoTree(pv);
                }
            } else {
                // 如果是最大层，则不需要
                if (update_enable_) {
                    new_points_num_++;
                    all_points_num_++;
                    if (update_cov_enable_) {
                        temp_points_.push_back(pv);
                    } else {
                        new_points_.push_back(pv);
                    }
                    if (new_points_num_ > update_size_threshold_) {
                        if (update_cov_enable_) {
                            init_plane(temp_points_, plane_ptr_);
                        } else {
                            update_plane(new_points_, plane_ptr_);
                            new_points_.clear();
                        }
                        new_points_num_ = 0;
                    }
                    if (all_points_num_ >= max_cov_points_size_) {
                        update_cov_enable_ = false;
                        std::vector<PointWithCov>().swap(temp_points_);
                    }
                    if (all_points_num_ >= max_points_size_) {
                        update_enable_ = false;
                        plane_ptr_->update_enable = false;
                        std::vector<PointWithCov>().swap(new_points_);
                    }
                }
            }
        }
    }
}

// 构建地图
void buildVoxelMap(const std::vector<PointWithCov>& input_points, const float voxel_size, const int max_layer,
                   const std::vector<int>& layer_point_size, const int max_points_size, const int max_cov_points_size,
                   const float plane_thresh, std::unordered_map<VOXEL_KEY, OctoTree*>& feat_map) {
    uint point_size = input_points.size();
    // 遍历所有的点，计算体素坐标
    for (uint i = 0; i < point_size; ++i) {
        const PointWithCov p_v = input_points[i];
        float loc_xyz[3];
        // 计算索引
        for (int j = 0; j < 3; ++j) {
            loc_xyz[j] = p_v.point[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_KEY position(static_cast<int64_t>(loc_xyz[0]), static_cast<int64_t>(loc_xyz[1]),
                           static_cast<int64_t>(loc_xyz[2]));
        auto iter = feat_map.find(position);
        // 不存在该体素
        if (iter == feat_map.end()) {
            feat_map[position]->temp_points_.push_back(p_v);
            feat_map[position]->new_points_num_++;
        } else {
            // 创建octotree
            OctoTree* octo_tree =
                new OctoTree(max_layer, 0, layer_point_size, max_points_size, max_cov_points_size, plane_thresh);
            // 设置八叉树相关的变量
            feat_map[position] = octo_tree;
            feat_map[position]->quater_length_ = voxel_size / 4;
            // 体素的中心
            feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            feat_map[position]->temp_points_.push_back(p_v);
            feat_map[position]->new_points_num_++;
            feat_map[position]->layer_point_size_ = layer_point_size;
        }
    }
    // 遍历所有的feat_map
    for (auto it = feat_map.begin(); it != feat_map.end(); ++it) {
        // 尝试初始化八叉树节点
        it->second->init_octo_tree();
    }
}

void updateVoxelMapOMP(const std::vector<PointWithCov>& input_points, const float voxel_size, const int max_layer,
                       const std::vector<int>& layer_point_size, const int max_points_size,
                       const int max_cov_points_size, const float plane_thresh,
                       std::unordered_map<VOXEL_KEY, OctoTree*>& feat_map) {
    std::unordered_map<VOXEL_KEY, std::vector<PointWithCov>> position_updates;
    int insert_num = 0, update_num = 0;
    uint point_size = input_points.size();
    auto start_time = std::chrono::high_resolution_clock::now();
    for (uint i = 0; i < point_size; ++i) {
        const PointWithCov p_v = input_points[i];
        float loc_xyz[3];
        // 计算索引
        for (int j = 0; j < 3; ++j) {
            loc_xyz[j] = p_v.point[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_KEY position(static_cast<int64_t>(loc_xyz[0]), static_cast<int64_t>(loc_xyz[1]),
                           static_cast<int64_t>(loc_xyz[2]));
        auto iter = feat_map.find(position);
        if (iter == feat_map.end()) {
            // 插入新的点，插入的点总是少的
            insert_num++;
            // 创建树节点
            OctoTree* octo_tree =
                new OctoTree(max_layer, 0, layer_point_size, max_points_size, max_cov_points_size, plane_thresh);
            feat_map[position] = octo_tree;
            feat_map[position]->quater_length_ = voxel_size / 4;
            // 体素的中心
            feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
            feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
            feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
            feat_map[position]->UpdateOctoTree(p_v);
        } else {
            update_num++;
            position_updates[position].push_back(p_v);
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>((end_time - start_time)).count();
    LOG_INFO("insert voxel use time:{}", duration * 1e-3);
    start_time = std::chrono::high_resolution_clock::now();
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for default(none) shared(position_updates, feat_map);
#endif
    //  先遍历bucket 理想情况下bucket一般只有一个元素 这样还是相当于完全并行的遍历position_index_map
    for (size_t b = 0; b < position_updates.bucket_count(); b++) {
        for (auto bi = position_updates.begin(b); bi != position_updates.end(b); ++bi) {
            VOXEL_KEY position = bi->first;
            for (const auto& pv : bi->second) {
                feat_map[position]->UpdateOctoTree(pv);
            }
        }
    }
    end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>((end_time - start_time)).count();
    LOG_INFO("update voxel use time:{}", duration * 1e-3);
    LOG_INFO("update voxel size:{}, insert voxel_size:{}", update_num, insert_num);
}

void BuildResidualListOMP(const std::unordered_map<VOXEL_KEY, OctoTree*>& voxel_map, const double voxel_size,
                          const double sigma_num, const int max_layer, const std::vector<PointWithCov>& pv_list,
                          std::vector<ptpl>& ptpl_list, std::vector<V3D>& non_match) {
    // 利用omp库加速
    std::mutex lock;
    ptpl_list.clear();
    std::vector<ptpl> all_ptpl_list(pv_list.size());
    std::vector<bool> useful_ptpl(pv_list.size());
    std::vector<size_t> index(pv_list.size());
    for (size_t i = 0; i < pv_list.size(); ++i) {
        index[i] = i;
        useful_ptpl[i] = false;
    }
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    // 遍历所有的点
    for (int i = 0; i < index.size(); ++i) {
        // 计算体素position
        PointWithCov pv = pv_list[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pv.point_wolrd[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_KEY position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        // 查找当前点所属的voxel
        auto iter = voxel_map.find(position);
        if (iter != voxel_map.end()) {
            OctoTree* current_octo = iter->second;
            ptpl single_ptpl;
            bool is_success;
            double prob = 0;
            // 构建残差
            build_single_residual(pv, current_octo, 0, max_layer, sigma_num, is_success, prob, single_ptpl);
            if (!is_success) {
                VOXEL_KEY near_position = position;
                if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
                    near_position.x = near_position.x + 1;
                } else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) {
                    near_position.x = near_position.x - 1;
                }
                if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
                    near_position.y = near_position.y + 1;
                } else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) {
                    near_position.y = near_position.y - 1;
                }
                if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
                    near_position.z = near_position.z + 1;
                } else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = voxel_map.find(near_position);
                if (iter_near != voxel_map.end()) {
                    build_single_residual(pv, iter_near->second, 0, max_layer, sigma_num, is_success, prob,
                                          single_ptpl);
                }
            }
            if (is_success) {
                lock.lock();
                useful_ptpl[i] = true;
                all_ptpl_list[i] = single_ptpl;
                lock.unlock();
            } else {
                // 不在当前的体素地图中
                lock.lock();
                useful_ptpl[i] = false;
                lock.unlock();
            }
        }
    }
    for (size_t i = 0; i < useful_ptpl.size(); i++) {
        if (useful_ptpl[i]) {
            ptpl_list.push_back(all_ptpl_list[i]);
        }
    }
}

void build_single_residual(const PointWithCov& pv, const OctoTree* current_octo, const int current_layer,
                           const int max_layer, const double sigma_num, bool& is_success, double& prob,
                           ptpl& single_ptpl) {
    // 构建点面残差
    double radius_k = 3;
    V3D p_w = pv.point_wolrd;
    // 如果当前节点拟合平面成功，则直接在这一层构建残差
    if (current_octo->plane_ptr_->is_plane) {
        Plane& plane = *current_octo->plane_ptr_;
        // 计算点到平面的距离
        float dis_to_plane =
            fabs(plane.normal(0) * p_w(0) + plane.normal(1) * p_w(1) + plane.normal(2) * p_w(2) + plane.d);
        // 计算点到平面中心簇的距离
        float dis_to_center = (plane.center(0) - p_w(0)) * (plane.center(0) - p_w(0)) +
                              (plane.center(1) - p_w(1)) * (plane.center(1) - p_w(1)) +
                              (plane.center(2) - p_w(2)) * (plane.center(2) - p_w(2));
        // HACK 差值是 点在平面上投影 与 平面点簇中心的距离
        // HACK 目的是不要用距离平面点簇太远的点来做残差，因为估计的平面在这些远点的位置可能不满足平面假设了
        // HACK 因为将点划分进voxel的时候只用了第一层voxel 这个voxel可能比较大 遍历到的这个子voxel距离点可能还比较远
        float range_dist = sqrt(dis_to_center - dis_to_plane * dis_to_plane);  // 三角形的底边
        // 选取底边不要那么远的点
        if (range_dist <= radius_k * plane.radius) {
            // 计算点面距离的方差
            Eigen::Matrix<double, 1, 6> J_np;
            J_np.block<1, 3>(0, 0) = p_w - plane.center;
            J_np.block<1, 3>(0, 3) = -plane.normal;
            double sigma_l = J_np * plane.plane_cov * J_np.transpose();
            sigma_l += plane.normal.transpose() * pv.cov * plane.normal;
            if (dis_to_plane < sigma_num * sqrt(sigma_l)) {
                is_success = true;
                double this_prob = 1.0 / sqrt(sigma_l) * exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
                if (this_prob > prob) {
                    prob = this_prob;
                    single_ptpl.point = pv.point;
                    single_ptpl.point_world = pv.point_wolrd;
                    single_ptpl.plane_cov = plane.plane_cov;
                    single_ptpl.nomral = plane.normal;
                    single_ptpl.center = plane.center;
                    single_ptpl.d = plane.d;
                    single_ptpl.layer = current_layer;
                    single_ptpl.cov_lidar = pv.cov_lidar;
                }
                return;
            } else {
                return;
            }
        } else {
            return;
        }
    } else {
        // 往层接更深的地方寻找平面
        if (current_layer < max_layer) {
            for (size_t leafnum = 0; leafnum < 8; ++leafnum) {
                if (current_octo->leaves_[leafnum] != nullptr) {
                    OctoTree* leaf_octo_tree = current_octo->leaves_[leafnum];
                    // 递归
                    build_single_residual(pv, leaf_octo_tree, current_layer + 1, max_layer, sigma_num, is_success, prob,
                                          single_ptpl);
                }
            }
            return;
        } else {
            return;
        }
    }
}
}  // namespace slam