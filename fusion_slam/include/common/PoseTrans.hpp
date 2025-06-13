#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <ostream>
namespace slam {

template <typename Scalar>
class PoseTranse {
   public:
    using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
    using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
    using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
    using Quaternion = Eigen::Quaternion<Scalar>;

    // 默认构造函数
    PoseTranse() : Rot(Eigen::Matrix<Scalar, 3, 3>::Identity()), trans(Eigen::Matrix<Scalar, 3, 1>::Zero()) {
    }

    PoseTranse(const Eigen::Matrix<Scalar, 3, 3>& rot, const Eigen::Matrix<Scalar, 3, 1>& t) : Rot(rot), trans(t) {
    }
    // copy和赋值
    PoseTranse(const PoseTranse& other) {
        Rot = other.Rot;
        trans = other.trans;
    }

    PoseTranse& operator=(const PoseTranse& other) {
        Rot = other.Rot;
        trans = other.trans;
        return *this;
    }
    // 移动/移动赋值
    PoseTranse(PoseTranse&& other) {
        Rot = std::move(other.Rot);
        trans = std::move(other.trans);
    }
    PoseTranse& operator=(PoseTranse&& other) {
        if (this != other) {
            Rot = std::move(other.Rot);
            trans = std::move(other.trans);
        }
        return *this;
    }

    // 重载乘法
    // 末尾的const作用是 保证了 const PoseTrans a * PoseTrans b可以通过编译
    // 并且不会修改this对象的成员
    PoseTranse& operator*(const PoseTranse& other) const {
        return PoseTranse(Rot * other.Rot, Rot * other.trans + trans);
    }
    // 点的乘法
    Vector3& operator*(const Vector3& origin_point) const {
        // 点变换
        return Rot * origin_point + trans;
    }

    // 逆变换
    PoseTranse inverse() const {
        return PoseTrans(Rot.inverse(), -Rot.inverse() * trans);
    }

    // 转换为齐次矩阵
    Matrix4 matrix() const {
        Matrix4 ret = Matrix4::Identity();
        ret.block<3, 3>(0, 0) = Rot;
        ret.block<3, 1>(0, 3) = trans;
        return ret;
    }

    // 获取四元数表示
    Quaternion eigen_q() const {
        return Quaternion(Rot);
    }

    // 计算RPY欧拉角
    Vector3 RPY() const {
        Vector3 rpy;
        rpy(1) = asin(Rot(2, 0));
        Scalar c_pitch = cos(rpy(1));
        rpy(0) = atan2(Rot(2, 1) / c_pitch, Rot(2, 2) / c_pitch);
        rpy(2) = atan2(Rot(1, 0) / c_pitch, Rot(0, 0) / c_pitch);
        return rpy;
    }

    // 计算范数
    Scalar norm_dist() const {
        return trans.norm();
    }

    Scalar norm_rot() const {
        return RPY().norm();
    }

    // 友元的重载输出
    friend std::ostream& operator<<(std::ostream& os, const PoseTranse& pose) {
        Eigen::Vector3d rpy = pose.RPY().template cast<double>();
        os << "x: " << static_cast<double>(pose.trans[0]) << " y: " << static_cast<double>(pose.trans[1])
           << " z: " << static_cast<double>(pose.trans[2]) << " roll: " << rpy[0] << " pitch: " << rpy[1]
           << " yaw: " << rpy[2];
        return os;
    }

    Matrix3 Rot;
    Vector3 trans;
};

using PoseTranseD = PoseTranse<double>;
using PoseTranseF = PoseTranse<float>;

}  // namespace slam