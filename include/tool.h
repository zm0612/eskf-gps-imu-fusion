//
// Created by meng on 2021/2/26.
//

#ifndef GPS_IMU_FUSION_TOOL_H
#define GPS_IMU_FUSION_TOOL_H

#include <eigen3/Eigen/Dense>

inline void TransformCoordinate(Eigen::Vector3d& vec){
    double kDegree2Radian = M_PI / 180.0;

    Eigen::Quaterniond Q_b_w = Eigen::AngleAxisd(90 * kDegree2Radian, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(0 * kDegree2Radian, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(180 * kDegree2Radian, Eigen::Vector3d::UnitX());

    vec = Q_b_w.inverse() * vec;
}
#endif //GPS_IMU_FUSION_TOOL_H
