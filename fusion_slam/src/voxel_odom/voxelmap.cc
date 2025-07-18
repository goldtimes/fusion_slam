#include "voxel_odom/voxelmap.hh"

namespace slam {
OctoTree::OctoTree(int _max_layer, int _layer, std::vector<int> _update_size_threshes, int _max_point_thresh,
                   double _plane_thresh)
    : max_layer_(_max_layer),
      current_layer_(_layer),
      max_layer_points_(_update_size_threshes),
      max_point_thresh_(_max_point_thresh),
      plane_thresh_(_plane_thresh) {
    tmp_points_.clear();
    is_leave_ = false;
    all_point_num = 0;
    new_point_num = 0;
    update_size_thresh_for_new = 5;
    is_initialized_ = false;
    update_enable_ = true;
    update_size_thresh_ = max_layer_points_[current_layer_];
    plane_.is_valid = false;
    leaves_.resize(8, nullptr);
}

void OctoTree::insert(const std::vector<PointWithCov> &input_points) {
    if (!is_initialized_) {
        // tmp_points_ 保存了体素中所有的点
        tmp_points_.insert(tmp_points_.end(), input_points.begin(), input_points.end());
        all_point_num += input_points.size();
        new_point_num += input_points.size();
        initialize_tree();
        return;
    }
    // 如果是叶子节点
    if (is_leave_) {
        if (update_enable_) {
            // 如果还能更新
            tmp_points_.insert(tmp_points_.end(), input_points.begin(), input_points.end());
            all_point_num += input_points.size();
            new_point_num += input_points.size();
            if (new_point_num >= update_size_thresh_for_new) {
                // 将全量的点，重新估计平面
                build_plane(tmp_points_);
                new_point_num = 0;
            }
            if (all_point_num > max_point_thresh_) {
                update_enable_ = false;
                std::vector<PointWithCov>().swap(tmp_points_);
            }
        }
        return;
    }
    // 还没有达到最大层且平面没有估计成功
    if (current_layer_ < max_layer_ - 1) {
        // 需要计算点在子体素中的key
        if (!tmp_points_.empty()) {
            std::vector<PointWithCov>().swap(tmp_points_);
        }
        // 8个子节点的poinst容器
        std::vector<std::vector<PointWithCov>> package(8, std::vector<PointWithCov>(0));
        for (int i = 0; i < input_points.size(); ++i) {
            int xyz[3]{0, 0, 0};
            int leafnum = subIndex(input_points[i], xyz);
            if (leaves_[leafnum] == nullptr) {
                leaves_[leafnum] = std::make_shared<OctoTree>(max_layer_, current_layer_ + 1, max_layer_points_,
                                                              max_point_thresh_, plane_thresh_);
                Eigen::Vector3d shift((2 * xyz[0] - 1) * quater_length, (2 * xyz[1] - 1) * quater_length,
                                      (2 * xyz[2] - 1) * quater_length);
                leaves_[leafnum]->voxel_center_ = voxel_center_ + shift;
                leaves_[leafnum]->quater_length = quater_length / 2;
            }
            package[leafnum].push_back(input_points[i]);
        }
        for (int i = 0; i < 8; i++) {
            if (package[i].size() == 0) {
                continue;
            }
            leaves_[i]->insert(package[i]);
        }
    }
}

void OctoTree::build_plane(const std::vector<PointWithCov> &points) {
    plane_.plane_cov = Mat6d::Zero();
    plane_.covariance = Eigen::Matrix3d::Zero();
    plane_.center = Eigen::Vector3d::Zero();
    plane_.normal = Eigen::Vector3d::Zero();
    plane_.points_size = points.size();
    for (auto &pt : points) {
        plane_.covariance += pt.point * pt.point.transpose();
        plane_.center += pt.point;
    }
    plane_.center = plane_.center / plane_.points_size;
    // 平面的方差，利用这个来主成分分析得到特征值最小的特征向量
    plane_.covariance = plane_.covariance / plane_.points_size - plane_.center * plane_.center.transpose();

    Eigen::EigenSolver<Eigen::Matrix3d> es(plane_.covariance);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal = evals.real();
    Eigen::Matrix3d::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;

    Eigen::Matrix3d J_Q = Eigen::Matrix3d::Identity() / static_cast<double>(plane_.points_size);
    plane_.eigens << evalsReal(evalsMin), evalsReal(evalsMid), evalsReal(evalsMax);
    plane_.normal = evecs.real().col(evalsMin);
    plane_.x_normal = evecs.real().col(evalsMax);
    plane_.y_normal = evecs.real().col(evalsMid);
    // 如果最小的特征向量小于平面阈值
    if (plane_.eigens[0] < plane_thresh_) {
        // 根据所有的点，拟合出平面的协方差
        for (int i = 0; i < points.size(); i++) {
            Eigen::Matrix<double, 6, 3> J;
            Eigen::Matrix3d F;
            for (int m = 0; m < 3; m++) {
                if (m != (int)evalsMin) {
                    Eigen::Matrix<double, 1, 3> F_m = (points[i].point - plane_.center).transpose() /
                                                      ((plane_.points_size) * (evalsReal[evalsMin] - evalsReal[m])) *
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
            plane_.plane_cov += J * points[i].cov * J.transpose();
        }
        plane_.is_valid = true;
    } else {
        plane_.is_valid = false;
    }
}

void OctoTree::initialize_tree() {
    // < 5个点
    if (all_point_num < update_size_thresh_) {
        return;
    }
    is_initialized_ = true;
    new_point_num = 0;
    build_plane(tmp_points_);
    if (plane_.is_valid) {
        is_leave_ = true;
        if (tmp_points_.size() > max_point_thresh_) {
            // 标记为不需要更新了
            update_enable_ = false;
            // 清空
            std::vector<PointWithCov>().swap(tmp_points_);
        } else {
            update_enable_ = true;
        }
    } else {
        // cut octo tree
        split_tree();
    }
}

void OctoTree::split_tree() {
    if (current_layer_ >= max_layer_ - 1) {
        is_leave_ = true;
        return;
    }

    std::vector<std::vector<PointWithCov>> package(8, std::vector<PointWithCov>(0));

    for (size_t i = 0; i < tmp_points_.size(); i++) {
        int xyz[3] = {0, 0, 0};
        int leafnum = subIndex(tmp_points_[i], xyz);
        if (leaves_[leafnum] == nullptr) {
            leaves_[leafnum] = std::make_shared<OctoTree>(max_layer_, current_layer_ + 1, max_layer_points_,
                                                          max_point_thresh_, plane_thresh_);
            Eigen::Vector3d shift((2 * xyz[0] - 1) * quater_length, (2 * xyz[1] - 1) * quater_length,
                                  (2 * xyz[2] - 1) * quater_length);
            leaves_[leafnum]->voxel_center_ = voxel_center_ + shift;
            leaves_[leafnum]->quater_length = quater_length / 2;
        }
        package[leafnum].push_back(tmp_points_[i]);
    }

    for (int i = 0; i < 8; i++) {
        if (package[i].size() == 0) continue;
        leaves_[i]->insert(package[i]);
    }
    std::vector<PointWithCov>().swap(tmp_points_);
}

// 判断点是否插入还是更新的点
void VoxelMap::pack(const std::vector<PointWithCov> &input_points) {
    sub_map.clear();
    int plsize = input_points.size();
    for (int i = 0; i < plsize; ++i) {
        const PointWithCov &pv = input_points[i];
        VOXEL_KEY key = calc_index(pv.point);
        auto it = feat_map.find(key);
        auto sub_it = sub_map.find(key);
        if (it != feat_map.end()) {
            // 如果改体素已经存在了
            if (!it->second.tree->update_enable_) {
                // 如果改节点不在被更新了
                continue;
            }
            if (sub_it == sub_map.end()) {
                // 标记为更新的体素
                sub_map[key].type = VOXEL_STATE::UPDATE;
                sub_map[key].it = feat_map[key].it;
            }
            sub_map[key].points.push_back(pv);

        } else {
            // 插入
            if (sub_it == sub_map.end()) {
                sub_map[key].type = VOXEL_STATE::INSERT;
            }
            sub_map[key].points.push_back(pv);
        }
    }
}

int OctoTree::subIndex(const PointWithCov &pv, int *xyz) {
    if (pv.point[0] > voxel_center_[0]) xyz[0] = 1;
    if (pv.point[1] > voxel_center_[1]) xyz[1] = 1;
    if (pv.point[2] > voxel_center_[2]) xyz[2] = 1;
    return 4 * xyz[0] + 2 * xyz[1] + xyz[2];
}

void VoxelMap::insert(const std::vector<PointWithCov> &input_points) {
    pack(input_points);
    // 遍历submap
    for (auto &pair : sub_map) {
        if (pair.second.type == VOXEL_STATE::INSERT) {
            // 在链表头插入体素的索引
            cache.push_front(pair.first);
            pair.second.it = cache.begin();
            // 创建octotree
            feat_map[pair.first].tree =
                std::make_shared<OctoTree>(max_layer, 0, max_layer_points, max_point_thresh, plane_thresh);
            // 保存链表的迭代器位置
            feat_map[pair.first].it = pair.second.it;
            feat_map[pair.first].tree->voxel_center_ =
                V3D((0.5 + pair.first.x) * voxel_size, (0.5 + pair.first.y) * voxel_size,
                    (0.5 + pair.first.y) * voxel_size);
            feat_map[pair.first].tree->quater_length = voxel_size / 4;
            feat_map[pair.first].tree->insert(pair.second.points);
            if (cache.size() > capacity) {
                feat_map.erase(cache.back());
                cache.pop_back();
            }
        } else {
            // 目标位置，源容器，要移动的元素
            cache.splice(cache.begin(), cache, pair.second.it);
            feat_map[pair.first].tree->insert(pair.second.points);
        }
    }
}

VOXEL_KEY VoxelMap::calc_index(const Eigen::Vector3d &point) {
    // 原版的对负值做了特别的处理，但是eigen有这样的函数
    V3D idx = (point / voxel_size).array().floor();
    return VOXEL_KEY(static_cast<int64_t>(idx[0]), static_cast<int64_t>(idx[1]), static_cast<int64_t>(idx[2]));
}

void VoxelMap::buildResidual(ResidualData &info, std::shared_ptr<OctoTree> oct_tree) {
    if (oct_tree->plane_.is_valid) {
        Eigen::Vector3d p_world_to_center = info.point_world - oct_tree->plane_.center;
        info.plane_center = oct_tree->plane_.center;
        info.plane_norm = oct_tree->plane_.normal;
        info.plane_cov = oct_tree->plane_.plane_cov;
        info.residual = info.plane_norm.transpose() * p_world_to_center;
        double dis_to_plane = std::abs(info.residual);
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = p_world_to_center;
        J_nq.block<1, 3>(0, 3) = -info.plane_norm;
        double sigma_l = J_nq * info.plane_cov * J_nq.transpose();
        sigma_l += info.plane_norm.transpose() * info.cov * info.plane_norm;
        if (dis_to_plane < info.sigma_num * sqrt(sigma_l)) {
            info.is_valid = true;
        }
    } else {
        if (info.current_layer < max_layer - 1) {
            for (size_t i = 0; i < 8; i++) {
                if (oct_tree->leaves_[i] == nullptr) continue;
                info.current_layer += 1;
                buildResidual(info, oct_tree->leaves_[i]);
                if (info.is_valid) break;
            }
        }
    }
}

}  // namespace slam