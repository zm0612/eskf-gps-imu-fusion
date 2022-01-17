//
// Created by meng on 2021/2/24.
//
#include "eskf_flow.h"
#include "tool.h"

#include <fstream>
#include <yaml-cpp/yaml.h>
Eigen::Matrix4d Vector2Matrix(const Eigen::Vector3d& vec){
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3,1>(0,3) = vec;

    return matrix;
}

ESKFFlow::ESKFFlow(const std::string &work_space_path)
        : work_space_path_(work_space_path){

    std::string config_file_path = work_space_path_ + "/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    eskf_ptr_ = std::make_shared<ESKF>(config_node);
}

bool ESKFFlow::ReadData() {
    const std::string data_path = work_space_path_ + "/data/raw_data";

    if (imu_flow_ptr_->ReadIMUData(data_path, imu_data_buff_) &&
        gps_flow_ptr_->ReadGPSData(data_path, gps_data_buff_)
    ) {
        return false;
    }

    return false;
}

bool ESKFFlow::ValidGPSAndIMUData() {
    curr_imu_data_ = imu_data_buff_.front();
    curr_gps_data_ = gps_data_buff_.front();

    double delta_time = curr_imu_data_.time - curr_gps_data_.time;

    if (delta_time > 0.01){
        gps_data_buff_.pop_front();
        return false;
    }

    if (delta_time < -0.01){
        imu_data_buff_.pop_front();
        return false;
    }

    imu_data_buff_.pop_front();
    gps_data_buff_.pop_front();

    return true;
}

bool ESKFFlow::ValidIMUData() {
    curr_imu_data_ = imu_data_buff_.front();
    imu_data_buff_.front();

    return true;
}

bool ESKFFlow::ValidGPSData() {
    curr_gps_data_ = gps_data_buff_.front();
    gps_data_buff_.pop_front();

    return true;
}

bool ESKFFlow::Run() {
    ReadData();

    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()){
        if (!ValidGPSAndIMUData()){
            continue;
        } else{
            eskf_ptr_->Init(curr_gps_data_, curr_imu_data_);
            break;
        }
    }

    std::ofstream gt_file(work_space_path_+"/data/gt.txt", std::ios::trunc);
    std::ofstream fused_file(work_space_path_+"/data/fused.txt", std::ios::trunc);
    std::ofstream measured_file(work_space_path_+"/data/measured.txt", std::ios::trunc);

    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()){
        curr_imu_data_ = imu_data_buff_.front();
        curr_gps_data_ = gps_data_buff_.front();
        if (curr_imu_data_.time < curr_gps_data_.time){
            eskf_ptr_->Predict(curr_imu_data_);
            imu_data_buff_.pop_front();
        } else{
            eskf_ptr_->Predict(curr_imu_data_);
            imu_data_buff_.pop_front();

            eskf_ptr_->Correct(curr_gps_data_);

            SavePose(fused_file, eskf_ptr_->GetPose());
            SavePose(measured_file,Vector2Matrix(curr_gps_data_.position));

            SavePose(gt_file, Vector2Matrix(GPSFlow::LLA2ENU(curr_gps_data_.true_position_lla)));
            gps_data_buff_.pop_front();
        }

        if (use_observability_analysis_) {
            Eigen::Matrix<double, 15, 15> F;
            Eigen::Matrix<double, 3, 15> G;
            Eigen::Matrix<double, 3, 1> Y;
            eskf_ptr_->GetFGY(F, G, Y);
            observability_analysis.SaveFG(F, G, Y, curr_gps_data_.time);
        }
    }

    if (use_observability_analysis_) {
        observability_analysis.ComputeSOM();
        observability_analysis.ComputeObservability();
    }
}

bool ESKFFlow::TestRun() {
    ReadData();

    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()) {
        if (!ValidGPSAndIMUData()) {
            continue;
        } else {
            eskf_ptr_->Init(curr_gps_data_, curr_imu_data_);
            std::cout << "\ntime: " << curr_gps_data_.time << std::endl;
            std::cout << "vel: " << eskf_ptr_->GetVelocity().transpose() << std::endl;
            std::cout << "measure vel: " << curr_gps_data_.velocity.transpose() << std::endl;
            std::cout << "true vel: " << curr_gps_data_.true_velocity.transpose() << std::endl;
            std::cout << "time: " << curr_gps_data_.time << std::endl;
            break;
        }
    }

    std::ofstream gt_file(work_space_path_ + "/data/gt.txt", std::ios::trunc);
    std::ofstream fused_file(work_space_path_ + "/data/fused.txt", std::ios::trunc);
    std::ofstream measured_file(work_space_path_ + "/data/measured.txt", std::ios::trunc);

    while (!imu_data_buff_.empty() && !gps_data_buff_.empty()) {
        curr_imu_data_ = imu_data_buff_.front();
        curr_gps_data_ = gps_data_buff_.front();
            eskf_ptr_->Predict(curr_imu_data_);
            imu_data_buff_.pop_front();
            SavePose(fused_file, eskf_ptr_->GetPose());
    }
}

void ESKFFlow::SavePose(std::ofstream &ofs, const Eigen::Matrix4d &pose) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ofs << pose(i, j);

            if (i == 2 && j == 3) {
                ofs << std::endl;
            } else {
                ofs << " ";
            }
        }
    }
}

