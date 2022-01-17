//
// Created by meng on 2021/2/19.
//
#include "tool.h"
#include "gps_flow.h"

#include <iostream>
#include <fstream>

GeographicLib::LocalCartesian GPSFlow::geo_converter_{32.0, 120.0, 0.0};

Eigen::Vector3d GPSFlow::LLA2ENU(const Eigen::Vector3d &lla) {
    Eigen::Vector3d enu;
    //lla -> ENU frame
    geo_converter_.Forward(lla[0], lla[1], lla[2], enu[0], enu[1], enu[2]);
    return enu;
}

void GPSFlow::LLA2ENU(GPSData &gps_data) {
    //lla -> ENU frame
    geo_converter_.Forward(gps_data.position_lla.x(),
                           gps_data.position_lla.y(),
                           gps_data.position_lla.z(),
                           gps_data.position.x(),
                           gps_data.position.y(),
                           gps_data.position.z());
}

bool GPSFlow::ReadGPSData(const std::string &path, std::vector<GPSData>& gps_data_vec,const int skip_rows) {
    std::string gps_file_path = path +"/gps-0.csv";
    std::string ref_gps_file_path = path +"/ref_gps.csv";
    std::string time_file_path = path + "/gps_time.csv";
    std::ifstream gps_file(gps_file_path, std::ios::in);
    std::ifstream ref_gps_file(ref_gps_file_path, std::ios::in);
    std::ifstream gps_time_file(time_file_path, std::ios::in);

    if (!gps_file.is_open() || !ref_gps_file.is_open() || !gps_time_file.is_open()){
        std::cerr << "failure to open gps file" << std::endl;
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
           && std::getline(gps_time_file, gps_time_line))
    {
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

        TransformCoordinate(gps_data.velocity);

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

        TransformCoordinate(gps_data.true_velocity);

        LLA2ENU(gps_data);

        gps_data_vec.emplace_back(gps_data);
    }

    gps_time_file.close();
    ref_gps_file.close();
    ref_gps_file.close();

    return true;
}

bool GPSFlow::ReadGPSData(const std::string &path, std::deque<GPSData>& gps_data_vec,const int skip_rows) {
    std::string gps_file_path = path +"/gps-0.csv";
    std::string ref_gps_file_path = path +"/ref_gps.csv";
    std::string time_file_path = path + "/gps_time.csv";
    std::ifstream gps_file(gps_file_path, std::ios::in);
    std::ifstream ref_gps_file(ref_gps_file_path, std::ios::in);
    std::ifstream gps_time_file(time_file_path, std::ios::in);

    if (!gps_file.is_open() || !ref_gps_file.is_open() || !gps_time_file.is_open()){
        std::cerr << "failure to open gps file" << std::endl;
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
           && std::getline(gps_time_file, gps_time_line))
    {
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

        TransformCoordinate(gps_data.velocity);

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

        TransformCoordinate(gps_data.true_velocity);

        LLA2ENU(gps_data);

        gps_data_vec.emplace_back(gps_data);
    }

    gps_time_file.close();
    ref_gps_file.close();
    ref_gps_file.close();

    return true;
}
