//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_GPS_FLOW_H
#define GPS_IMU_FUSION_GPS_FLOW_H

#include "gps_data.h"
#include "GeographicLib/LocalCartesian.hpp"

#include <deque>
#include <vector>

class GPSFlow{
public:
    GPSFlow() = default;

    static void LLA2NED(GPSData& gps_data);

    static Eigen::Vector3d LLA2NED(const Eigen::Vector3d& lla);

    static bool ReadGPSData(const std::string& path, std::vector<GPSData>& gps_data_vec,const int skip_rows=1);

    static bool ReadGPSData(const std::string& path, std::deque<GPSData>& gps_data_vec,const int skip_rows=1);

private:
    static GeographicLib::LocalCartesian geo_converter_;
};

#endif //GPS_IMU_FUSION_GPS_FLOW_H
