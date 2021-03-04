//
// Created by meng on 2021/2/23.
//

#include <iostream>

#include "global_defination.h"
#include "imu_flow.h"

int main(){
    std::string data_path = WORK_SPACE_PATH + "/data/raw_data";

    std::vector<IMUData> imu_data_buff;

    IMUFlow imu_flow;

    imu_flow.ReadIMUData(data_path, imu_data_buff);

    for (int i = 0; i < imu_data_buff.size(); ++i) {
        std::cout << "\nindex: " << i << std::endl;
        std::cout << "time: " << std::to_string(imu_data_buff.at(i).time) << std::endl;
        std::cout << "ref_gyro: " << imu_data_buff.at(i).true_angle_velocity.transpose() << std::endl;
        std::cout << "gyro: " << imu_data_buff.at(i).angle_velocity.transpose() << std::endl;
        std::cout << "accel: " << imu_data_buff.at(i).linear_accel.transpose() << std::endl;
    }

    return 0;
}

