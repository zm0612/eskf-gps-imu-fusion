//
// Created by meng on 2021/2/23.
//

#include <iostream>

#include "global_defination.h"
#include "gps_flow.h"

int main(){
    std::string data_path = WORK_SPACE_PATH + "/data/raw_data";

    std::vector<GPSData> gps_data_buff;

    GPSFlow gps_flow;

    gps_flow.ReadGPSData(data_path, gps_data_buff);


    for (int i = 0; i < gps_data_buff.size(); ++i) {
        std::cout << "\nindex: " << i << std::endl;
        std::cout << "time:" << std::to_string(gps_data_buff.at(i).time) << std::endl;
        std::cout << "posi(LLA): " << gps_data_buff.at(i).position_lla.transpose() << std::endl;
        std::cout << "posi(NED): " << gps_data_buff.at(i).position_ned.transpose() << std::endl;
        std::cout << "velo: " << gps_data_buff.at(i).velocity.transpose() << std::endl;
        std::cout << "true velo" << gps_data_buff.at(i).true_velocity.transpose() << std::endl;
    }

    return 0;
}

