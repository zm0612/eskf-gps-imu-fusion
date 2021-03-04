//
// Created by meng on 2021/2/19.
//
#include "tool.h"
#include "imu_flow.h"
#include <fstream>
#include <iostream>

bool IMUFlow::ReadIMUData(const std::string &path, std::vector<IMUData> &imu_data_buff, const int skip_rows) {
    constexpr double D2R = 0.017453292519943295;

    std::string accel_file_path = path + "/accel-0.csv";
    std::string ref_accel_file_path = path + "/ref_accel.csv";
    std::string gyro_file_path = path + "/gyro-0.csv";
    std::string ref_gyro_file_path = path + "/ref_gyro.csv";
    std::string time_file_path = path + "/time.csv";

    std::ifstream accel_file(accel_file_path);
    std::ifstream ref_accel_file(ref_accel_file_path);
    std::ifstream gyro_file(gyro_file_path);
    std::ifstream ref_gyro_file(ref_gyro_file_path);
    std::ifstream time_file(time_file_path);

    if (!accel_file.is_open() || !ref_accel_file.is_open()
        || !gyro_file.is_open() || !ref_gyro_file.is_open()
        || !time_file.is_open())
    {
        std::cerr << "failure to open imu file" << std::endl;
    }

    IMUData imu_data;
    imu_data_buff.clear();

    std::string accel_line;
    std::string ref_accel_line;
    std::string gyro_line;
    std::string ref_gyro_line;
    std::string time_line;
    std::string temp;

    for (int i = 0; i < skip_rows; ++i) {
        std::getline(accel_file, temp);
        std::getline(ref_accel_file, temp);
        std::getline(gyro_file, temp);
        std::getline(ref_gyro_file, temp);
        std::getline(time_file, temp);
    }

    while ( std::getline(accel_file, accel_line) &&
            std::getline(ref_accel_file, ref_accel_line) &&
            std::getline(gyro_file, gyro_line) &&
            std::getline(ref_gyro_file, ref_gyro_line) &&
            std::getline(time_file, time_line))
    {
        imu_data.time = std::stod(time_line);

        std::stringstream ss;
        ss << accel_line;

        std::getline(ss, temp, ',');
        imu_data.linear_accel.x() = std::stod(temp);
        std::getline(ss, temp, ',');
        imu_data.linear_accel.y() = std::stod(temp);
        std::getline(ss, temp, ',');
        imu_data.linear_accel.z() = std::stod(temp);

        TransformCoordinate(imu_data.linear_accel);

        ss.clear();
        ss << ref_accel_line;
        std::getline(ss, temp, ',');
        imu_data.true_linear_accel.x() = std::stod(temp);
        std::getline(ss, temp, ',');
        imu_data.true_linear_accel.y() = std::stod(temp);
        std::getline(ss, temp, ',');
        imu_data.true_linear_accel.z() = std::stod(temp);

        TransformCoordinate(imu_data.true_angle_velocity);

        ss.clear();
        ss << gyro_line;
        std::getline(ss, temp, ',');
        imu_data.angle_velocity.x() = std::stod(temp) * D2R;
        std::getline(ss, temp, ',');
        imu_data.angle_velocity.y() = std::stod(temp) * D2R;
        std::getline(ss, temp, ',');
        imu_data.angle_velocity.z() = std::stod(temp) * D2R;

        TransformCoordinate(imu_data.angle_velocity);

        ss.clear();
        ss << ref_gyro_line;
        std::getline(ss, temp, ',');
        imu_data.true_angle_velocity.x() = std::stod(temp) * D2R;
        std::getline(ss, temp, ',');
        imu_data.true_angle_velocity.y() = std::stod(temp) * D2R;
        std::getline(ss, temp, ',');
        imu_data.true_angle_velocity.z() = std::stod(temp) * D2R;

        TransformCoordinate(imu_data.true_angle_velocity);

        imu_data_buff.emplace_back(imu_data);
    }

    return true;
}

bool IMUFlow::ReadIMUData(const std::string &path, std::deque<IMUData> &imu_data_buff, const int skip_rows) {
    constexpr double D2R = 0.017453292519943295;

    std::string accel_file_path = path + "/accel-0.csv";
    std::string ref_accel_file_path = path + "/ref_accel.csv";
    std::string gyro_file_path = path + "/gyro-0.csv";
    std::string ref_gyro_file_path = path + "/ref_gyro.csv";
    std::string time_file_path = path + "/time.csv";

    std::ifstream accel_file(accel_file_path);
    std::ifstream ref_accel_file(ref_accel_file_path);
    std::ifstream gyro_file(gyro_file_path);
    std::ifstream ref_gyro_file(ref_gyro_file_path);
    std::ifstream time_file(time_file_path);

    if (!accel_file.is_open() || !ref_accel_file.is_open()
        || !gyro_file.is_open() || !ref_gyro_file.is_open()
        || !time_file.is_open())
    {
        std::cerr << "failure to open imu file" << std::endl;
    }

    IMUData imu_data;
    imu_data_buff.clear();

    std::string accel_line;
    std::string ref_accel_line;
    std::string gyro_line;
    std::string ref_gyro_line;
    std::string time_line;
    std::string temp;

    for (int i = 0; i < skip_rows; ++i) {
        std::getline(accel_file, temp);
        std::getline(ref_accel_file, temp);
        std::getline(gyro_file, temp);
        std::getline(ref_gyro_file, temp);
        std::getline(time_file, temp);
    }

    while ( std::getline(accel_file, accel_line) &&
            std::getline(ref_accel_file, ref_accel_line) &&
            std::getline(gyro_file, gyro_line) &&
            std::getline(ref_gyro_file, ref_gyro_line) &&
            std::getline(time_file, time_line))
    {
        imu_data.time = std::stod(time_line);

        std::stringstream ss;
        ss << accel_line;

        std::getline(ss, temp, ',');
        imu_data.linear_accel.x() = std::stod(temp);
        std::getline(ss, temp, ',');
        imu_data.linear_accel.y() = std::stod(temp);
        std::getline(ss, temp, ',');
        imu_data.linear_accel.z() = std::stod(temp);

        TransformCoordinate(imu_data.linear_accel);

        ss.clear();
        ss << ref_accel_line;
        std::getline(ss, temp, ',');
        imu_data.true_linear_accel.x() = std::stod(temp);
        std::getline(ss, temp, ',');
        imu_data.true_linear_accel.y() = std::stod(temp);
        std::getline(ss, temp, ',');
        imu_data.true_linear_accel.z() = std::stod(temp);

        TransformCoordinate(imu_data.true_linear_accel);

        ss.clear();
        ss << gyro_line;
        std::getline(ss, temp, ',');
        imu_data.angle_velocity.x() = std::stod(temp) * D2R;
        std::getline(ss, temp, ',');
        imu_data.angle_velocity.y() = std::stod(temp) * D2R;
        std::getline(ss, temp, ',');
        imu_data.angle_velocity.z() = std::stod(temp) * D2R;

        TransformCoordinate(imu_data.angle_velocity);

        ss.clear();
        ss << ref_gyro_line;
        std::getline(ss, temp, ',');
        imu_data.true_angle_velocity.x() = std::stod(temp) * D2R;
        std::getline(ss, temp, ',');
        imu_data.true_angle_velocity.y() = std::stod(temp) * D2R;
        std::getline(ss, temp, ',');
        imu_data.true_angle_velocity.z() = std::stod(temp) * D2R;

        TransformCoordinate(imu_data.true_angle_velocity);

        imu_data_buff.emplace_back(imu_data);
    }

    return true;
}