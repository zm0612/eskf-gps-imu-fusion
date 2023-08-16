//
// Created by meng on 2021/2/24.
//
#ifndef GPS_IMU_FUSION_ESKF_FLOW_H
#define GPS_IMU_FUSION_ESKF_FLOW_H

#include "eskf.h"
#include "imu_tool.h"
#include "gps_tool.h"
#include "config_parameters.h"
#include "observability_analysis.h"

#include <memory>
#include <deque>
#include <iostream>

class ESKFFlow {
public:
    ESKFFlow() = delete;

    explicit ESKFFlow(const std::string &config_file_path, std::string data_file_path);

    /*!
     * 从本地文件中读取IMU和GPS的数据
     * @return
     */
    void ReadData();

    /*!
     * 对IMU和GPS数据进行时间戳对齐，该函数只在ESKF初始化时使用
     * @return
     */
    bool ValidGPSAndIMUData();

    bool Run();

    bool TestRun();

    /*!
     * 保存位姿，为kitti格式
     * @param ofs
     * @param pose
     */
    static void SavePose(std::ofstream &ofs, const Eigen::Matrix4d &pose);

    /*!
     * Save TUM pose
     *
     * note:
     * timestamp x y z q_x q_y q_z q_w
     *
     * @param ofs
     * @param pose
     */
    static void SaveTUMPose(std::ofstream &ofs, const Eigen::Quaterniond &q,
                            const Eigen::Vector3d &t, double timestamp);

private:
    ConfigParameters config_parameters_;

    std::shared_ptr<ErrorStateKalmanFilter> eskf_ptr_;
    std::shared_ptr<IMUTool> imu_flow_ptr_;
    std::shared_ptr<GPSTool> gps_flow_ptr_;

    ObservabilityAnalysis observability_analysis;//可观测度分析工具

    std::deque<IMUData> imu_data_buff_;
    std::deque<GPSData> gps_data_buff_;

    IMUData curr_imu_data_;
    GPSData curr_gps_data_;

    bool use_observability_analysis_ = false;//是否进行可观测度分析

    const std::string config_file_path_;
    const std::string data_file_path_;
};

#endif //GPS_IMU_FUSION_ESKF_FLOW_H
