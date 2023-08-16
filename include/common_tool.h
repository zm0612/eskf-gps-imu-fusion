//
// Created by meng on 2021/2/26.
//

#ifndef GPS_IMU_FUSION_COMMON_TOOL_H
#define GPS_IMU_FUSION_COMMON_TOOL_H

#include <eigen3/Eigen/Dense>
#include <utility>
#include <iostream>

#define kDegree2Radian (M_PI / 180.0)

inline void TransformCoordinate(Eigen::Vector3d &vec) {
    Eigen::Quaterniond Q_b_w = Eigen::AngleAxisd(90 * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(180 * kDegree2Radian, Eigen::Vector3d::UnitX());

    vec = Q_b_w.inverse() * vec;
}

template<typename Scalar>
inline Eigen::Matrix<Scalar, 3, 1>
NedToEnu(const Eigen::Matrix<Scalar, 3, 1> &data) {
    Eigen::AngleAxis<Scalar> angle_axisd_x(static_cast<Scalar>(M_PI), Eigen::Matrix<Scalar, 3, 1>::UnitX());
    Eigen::AngleAxis<Scalar> angle_axisd_z(static_cast<Scalar>(M_PI_2), Eigen::Matrix<Scalar, 3, 1>::UnitZ());

    Eigen::Matrix<Scalar, 3, 3> R = angle_axisd_z.toRotationMatrix() * angle_axisd_x.toRotationMatrix();

    return R * data;
}

template<class Derived>
inline Derived NedToEnu(const Eigen::QuaternionBase<Derived> &q) {
    Eigen::AngleAxis<typename Derived::Scalar> angle_axisd_x(static_cast<typename Derived::Scalar>(M_PI),
                                                             Eigen::Matrix<typename Derived::Scalar, 3, 1>::UnitX());

    Eigen::AngleAxis<typename Derived::Scalar> angle_axisd_z(static_cast<typename Derived::Scalar>(M_PI_2),
                                                             Eigen::Matrix<typename Derived::Scalar, 3, 1>::UnitZ());
    Eigen::Matrix<typename Derived::Scalar, 3, 3> R =
            angle_axisd_z.toRotationMatrix() * angle_axisd_x.toRotationMatrix();

    Derived q_trans = Derived(R) * q;

    return q_trans;
}

//四元数 --> 欧拉角(Z-Y-X，即RPY)（确保pitch的范围[-pi/2, pi/2]）
inline Eigen::Vector3d Quaternion2EulerAngles(Eigen::Quaterniond q) {
    Eigen::Vector3d angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles(2) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    angles(0) = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

inline Eigen::Vector3d Rotation2EulerAngles(Eigen::Matrix3d R) {
//旋转矩阵 --> 欧拉角(Z-Y-X，即RPY)（确保pitch的范围[-pi/2, pi/2]）
    Eigen::Vector3d eulerAngle_mine;
    Eigen::Matrix3d rot = std::move(R);
    eulerAngle_mine(2) = std::atan2(rot(2, 1), rot(2, 2));
    eulerAngle_mine(1) = std::atan2(-rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
    eulerAngle_mine(0) = std::atan2(rot(1, 0), rot(0, 0));

    return eulerAngle_mine;
}

template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> BuildSkewSymmetricMatrix(const Eigen::MatrixBase<Derived> &vec) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> matrix;
    matrix << static_cast<typename Derived::Scalar>(0.0), -vec[2], vec[1],
            vec[2], static_cast<typename Derived::Scalar>(0.0), -vec[0],
            -vec[1], vec[0], static_cast<typename Derived::Scalar>(0.0);

    return matrix;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SO3Exp(const Eigen::MatrixBase<Derived> &v) {
    Eigen::Matrix<typename Derived::Scalar, 3, 3> R;
    typename Derived::Scalar theta = v.norm();
    Eigen::Matrix<typename Derived::Scalar, 3, 1> v_normalized = v.normalized();
    R = std::cos(theta) * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity()
        + (typename Derived::Scalar(1.0) - std::cos(theta)) * v_normalized *
          v_normalized.transpose() + std::sin(theta) * BuildSkewSymmetricMatrix(v_normalized);

    return R;
}

#endif //GPS_IMU_FUSION_COMMON_TOOL_H
