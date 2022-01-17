//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_GPS_DATA_H
#define GPS_IMU_FUSION_GPS_DATA_H

#include "GeographicLib/LocalCartesian.hpp"
#include <eigen3/Eigen/Core>

class GPSData{
public:
    GPSData() = default;

    Eigen::Vector3d position_lla = Eigen::Vector3d::Zero(); // LLA
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();     // ENU
    Eigen::Vector3d position = Eigen::Vector3d::Zero();     // ENU

    Eigen::Vector3d true_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d true_position_lla = Eigen::Vector3d::Zero();
    double time = 0.0;
};

#endif //GPS_IMU_FUSION_GPS_DATA_H
