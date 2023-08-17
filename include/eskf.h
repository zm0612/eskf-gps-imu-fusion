//
// Created by meng on 2021/2/19.
//

#ifndef GPS_IMU_FUSION_ESKF_H
#define GPS_IMU_FUSION_ESKF_H

#include "imu_data.h"
#include "gps_data.h"
#include "config_parameters.h"

#include <glog/logging.h>
#include <eigen3/Eigen/Dense>

#include <deque>

class ErrorStateKalmanFilter {
public:
    explicit ErrorStateKalmanFilter(const ConfigParameters &config_parameters);

    /*!
     * 用于ESKF滤波器的初始化，设置初始位姿，初始速度
     * @param curr_gps_data 与imu时间同步的gps数据
     * @param curr_imu_data 与gps时间同步的imu数据
     * @return
     */
    bool Init(const GPSData &curr_gps_data, const IMUData &curr_imu_data);

    /*!
     * IMU姿态解算，以及滤波器的预测步的构建，对应卡尔曼滤波器的前两个公式
     * @param curr_imu_data
     * @return
     */
    bool Predict(const IMUData &curr_imu_data);

    /*!
     * 滤波器的矫正，对应卡尔曼滤波器的后三个公式
     * @param curr_gps_data
     * @return
     */
    bool Correct(const GPSData &curr_gps_data);

    Eigen::Matrix4d GetPose() const;

    Eigen::Vector3d GetVelocity() {
        return velocity_;
    }

private:
    void SetCovarianceQ(double gyro_noise_cov, double accel_noise_cov);

    /*!
     *
     * @param position_x_std
     */
    void SetCovarianceR(double position_x_std, double position_y_std, double position_z_std);

    void SetCovarianceP(double posi_noise, double velocity_noise, double ori_noise,
                        double gyro_noise, double accel_noise);

    /*!
     * 通过IMU计算位姿和速度
     * @return
     */
    void UpdateOdomEstimation(const Eigen::Vector3d &w_in);

    void UpdateErrorState(double t, const Eigen::Vector3d &accel, const Eigen::Vector3d &w_in_n);

    Eigen::Vector3d ComputeDeltaRotation(const IMUData &imu_data_0, const IMUData &imu_data_1);

    /*!
     * 计算地球转动给导航系带来的变换
     * note: 对于普通消费级IMU可以忽略此项
     * @param R_nm_nm_1
     * @return
     */
    Eigen::Vector3d ComputeNavigationFrameAngularVelocity();

    /*!
     * 通过IMU计算当前姿态
     * @param angular_delta
     * @param R_nm_nm_1
     * @param curr_R
     * @param last_R
     * @return
     */
    void ComputeOrientation(const Eigen::Vector3d &angular_delta,
                            const Eigen::Matrix3d &R_nm_nm_1,
                            Eigen::Matrix3d &curr_R,
                            Eigen::Matrix3d &last_R);

    void ComputeVelocity(const Eigen::Matrix3d &R_0, const Eigen::Matrix3d &R_1, const IMUData &imu_data_0,
                         const IMUData &imu_data_1, Eigen::Vector3d &last_vel, Eigen::Vector3d &curr_vel);

    Eigen::Vector3d ComputeUnbiasAccel(const Eigen::Vector3d &accel);

    Eigen::Vector3d ComputeUnbiasGyro(const Eigen::Vector3d &gyro);

    /*!
     * 通过imu计算当前位移
     * @param curr_vel
     * @param last_vel
     * @return
     */
    void ComputePosition(const Eigen::Vector3d &last_vel, const Eigen::Vector3d &curr_vel, const IMUData &imu_data_0,
                         const IMUData &imu_data_1);

    /*!
     * 对误差进行滤波之后，需要在实际算出来的轨迹中，消除这部分误差
     */
    void EliminateError();

    /*!
     * 每次矫正之后，需要重置状态变量X
     */
    void ResetState();

private:
    static const unsigned int DIM_STATE = 15;
    static const unsigned int DIM_STATE_NOISE = 6;
    static const unsigned int DIM_MEASUREMENT = 3;
    static const unsigned int DIM_MEASUREMENT_NOISE = 3;

    static const unsigned int INDEX_STATE_POSI = 0;
    static const unsigned int INDEX_STATE_VEL = 3;
    static const unsigned int INDEX_STATE_ORI = 6;
    static const unsigned int INDEX_STATE_GYRO_BIAS = 9;
    static const unsigned int INDEX_STATE_ACC_BIAS = 12;
    static const unsigned int INDEX_MEASUREMENT_POSI = 0;

    typedef typename Eigen::Matrix<double, DIM_STATE, 1> TypeVectorX;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, 1> TypeVectorY;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixF;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> TypeMatrixB;
    typedef typename Eigen::Matrix<double, DIM_STATE_NOISE, DIM_STATE_NOISE> TypeMatrixQ;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixP;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_MEASUREMENT> TypeMatrixK;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, DIM_MEASUREMENT_NOISE> TypeMatrixC;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE> TypeMatrixG;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_MEASUREMENT> TypeMatrixR;

    TypeVectorX X_ = TypeVectorX::Zero();
    TypeVectorY Y_ = TypeVectorY::Zero();
    TypeMatrixF F_ = TypeMatrixF::Zero();
    TypeMatrixB B_ = TypeMatrixB::Zero();
    TypeMatrixQ Q_ = TypeMatrixQ::Zero();
    TypeMatrixP P_ = TypeMatrixP::Zero();
    TypeMatrixK K_ = TypeMatrixK::Zero();
    TypeMatrixC C_ = TypeMatrixC::Zero();
    TypeMatrixG G_ = TypeMatrixG::Zero();
    TypeMatrixC R_ = TypeMatrixR::Zero();

    TypeMatrixF Ft_ = TypeMatrixF::Zero();

    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();

    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias_ = Eigen::Vector3d::Zero();

    Eigen::Vector3d g_ = Eigen::Vector3d::Zero();//重力加速度

    GPSData curr_gps_data_;

    double earth_rotation_speed_{0.0};

    std::deque<IMUData> imu_data_buff_;

    ConfigParameters config_parameters_;
public:
    void GetFGY(TypeMatrixF &F, TypeMatrixG &G, TypeVectorY &Y);
};

#endif //GPS_IMU_FUSION_ESKF_H