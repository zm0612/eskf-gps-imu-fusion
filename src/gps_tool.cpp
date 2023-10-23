//
// Created by meng on 2021/2/19.
//
#include "common_tool.h"
#include "gps_tool.h"

#include <glog/logging.h>

#include <iostream>
#include <fstream>

GPSTool::GPSTool(double lon, double lat, double altitude) {
    GPSTool::geo_converter_.Reset(lat, lon, altitude);
}

Eigen::Vector3d GPSTool::LLAToLocalNED(const Eigen::Vector3d &lla) {
    Eigen::Vector3d enu;

    double enu_x, enu_y, enu_z;
    geo_converter_.Forward(lla.x(), lla.y(), lla.z(),
                           enu_x, enu_y, enu_z);
    enu.x() = enu_y;
    enu.y() = enu_x;
    enu.z() = -enu_z;

    return enu;
}

void GPSTool::LLAToLocalNED(GPSData &gps_data) {
    // LLA -> ENU frame
    double enu_x, enu_y, enu_z;
    geo_converter_.Forward(gps_data.position_lla.x(),
                           gps_data.position_lla.y(),
                           gps_data.position_lla.z(),
                           enu_x, enu_y, enu_z);

    // ENU -> NED
    gps_data.local_position_ned.x() = enu_y;
    gps_data.local_position_ned.y() = enu_x;
    gps_data.local_position_ned.z() = -enu_z;
}

void GPSTool::ReadGPSData(const std::string &path, std::vector<GPSData> &gps_data_vec, int skip_rows) {
    std::string gps_file_path = path + "/gps-0.csv";
    std::string ref_gps_file_path = path + "/ref_gps.csv";
    std::string time_file_path = path + "/gps_time.csv";
    std::ifstream gps_file(gps_file_path, std::ios::in);
    std::ifstream ref_gps_file(ref_gps_file_path, std::ios::in);
    std::ifstream gps_time_file(time_file_path, std::ios::in);

    if (!gps_file.is_open() || !ref_gps_file.is_open() || !gps_time_file.is_open()) {
        LOG(FATAL) << "failure to open gps file";
    }

    GPSData gps_data;
    gps_data_vec.clear();

    std::string gps_data_line;
    std::string ref_gps_data_line;
    std::string gps_time_line;
    std::string temp;

    for (int i = 0; i < skip_rows; ++i) {
        std::getline(gps_file, temp);
        std::getline(ref_gps_file, temp);
        std::getline(gps_time_file, temp);
    }

    while (std::getline(gps_file, gps_data_line)
           && std::getline(ref_gps_file, ref_gps_data_line)
           && std::getline(gps_time_file, gps_time_line)) {
        gps_data.time = std::stod(gps_time_line);

        std::stringstream ssr_0;
        std::stringstream ssr_1;

        ssr_0 << gps_data_line;
        ssr_1 << ref_gps_data_line;

        std::getline(ssr_0, temp, ',');
        gps_data.position_lla.x() = std::stod(temp);

        std::getline(ssr_0, temp, ',');
        gps_data.position_lla.y() = std::stod(temp);

        std::getline(ssr_0, temp, ',');
        gps_data.position_lla.z() = std::stod(temp);

        std::getline(ssr_0, temp, ',');
        gps_data.velocity.x() = std::stod(temp);

        std::getline(ssr_0, temp, ',');
        gps_data.velocity.y() = std::stod(temp);

        std::getline(ssr_0, temp, ',');
        gps_data.velocity.z() = std::stod(temp);

        std::getline(ssr_1, temp, ',');
        gps_data.true_position_lla.x() = std::stod(temp);

        std::getline(ssr_1, temp, ',');
        gps_data.true_position_lla.y() = std::stod(temp);

        std::getline(ssr_1, temp, ',');
        gps_data.true_position_lla.z() = std::stod(temp);

        std::getline(ssr_1, temp, ',');
        gps_data.true_velocity.x() = std::stod(temp);

        std::getline(ssr_1, temp, ',');
        gps_data.true_velocity.y() = std::stod(temp);

        std::getline(ssr_1, temp, ',');
        gps_data.true_velocity.z() = std::stod(temp);

        LLAToLocalNED(gps_data);

        gps_data_vec.emplace_back(gps_data);
    }

    gps_time_file.close();
    ref_gps_file.close();
    ref_gps_file.close();
}

void GPSTool::ReadGPSData(const std::string &path, std::deque<GPSData> &gps_data_vec, int skip_rows) {
    LOG(INFO) << "Read GPS data ...";
    std::string gps_file_path = path + "/gps-0.csv";
    std::string ref_gps_file_path = path + "/ref_gps.csv";
    std::string time_file_path = path + "/gps_time.csv";
    std::ifstream gps_file(gps_file_path, std::ios::in);
    std::ifstream ref_gps_file(ref_gps_file_path, std::ios::in);
    std::ifstream gps_time_file(time_file_path, std::ios::in);

    if (!gps_file.is_open() || !ref_gps_file.is_open() || !gps_time_file.is_open()) {
        LOG(FATAL) << "failure to open gps file";
    }

    GPSData gps_data;
    gps_data_vec.clear();

    std::string gps_data_line;
    std::string ref_gps_data_line;
    std::string gps_time_line;
    std::string temp;

    for (int i = 0; i < skip_rows; ++i) {
        std::getline(gps_file, temp);
        std::getline(ref_gps_file, temp);
        std::getline(gps_time_file, temp);
    }

    while (std::getline(gps_file, gps_data_line)
           && std::getline(ref_gps_file, ref_gps_data_line)
           && std::getline(gps_time_file, gps_time_line)) {
        gps_data.time = std::stod(gps_time_line);

        std::stringstream ssr_0;
        std::stringstream ssr_1;

        ssr_0 << gps_data_line;
        ssr_1 << ref_gps_data_line;

        std::getline(ssr_0, temp, ',');
        gps_data.position_lla.x() = std::stod(temp);

        std::getline(ssr_0, temp, ',');
        gps_data.position_lla.y() = std::stod(temp);

        std::getline(ssr_0, temp, ',');
        gps_data.position_lla.z() = std::stod(temp);

        std::getline(ssr_0, temp, ',');
        gps_data.velocity.x() = std::stod(temp);

        std::getline(ssr_0, temp, ',');
        gps_data.velocity.y() = std::stod(temp);

        std::getline(ssr_0, temp, ',');
        gps_data.velocity.z() = std::stod(temp);

        std::getline(ssr_1, temp, ',');
        gps_data.true_position_lla.x() = std::stod(temp);

        std::getline(ssr_1, temp, ',');
        gps_data.true_position_lla.y() = std::stod(temp);

        std::getline(ssr_1, temp, ',');
        gps_data.true_position_lla.z() = std::stod(temp);

        std::getline(ssr_1, temp, ',');
        gps_data.true_velocity.x() = std::stod(temp);

        std::getline(ssr_1, temp, ',');
        gps_data.true_velocity.y() = std::stod(temp);

        std::getline(ssr_1, temp, ',');
        gps_data.true_velocity.z() = std::stod(temp);

        LLAToLocalNED(gps_data);

        gps_data_vec.emplace_back(gps_data);
    }

    gps_time_file.close();
    ref_gps_file.close();
    ref_gps_file.close();

    LOG(INFO) << "Read GPS data successfully";
}