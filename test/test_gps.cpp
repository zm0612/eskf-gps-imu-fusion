//
// Created by meng on 2021/2/23.
//

#include <iostream>

#include "gps_tool.h"

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cout << "Please enter right command!" << std::endl;
        std::cout << "$ ./test_gps data_file_path" << std::endl;

        return -1;
    }

    std::string data_path = std::string(argv[1]);

    std::vector<GPSData> gps_data_buff;

    GPSTool gps_tool(0.0, 0.0, 0.0);
    gps_tool.ReadGPSData(data_path, gps_data_buff);


    for (int i = 0; i < gps_data_buff.size(); ++i) {
        std::cout << "\nindex: " << i << std::endl;
        std::cout << "time:" << std::to_string(gps_data_buff.at(i).time) << std::endl;
        std::cout << "posi(LLA): " << gps_data_buff.at(i).position_lla.transpose() << std::endl;
        std::cout << "posi(NED): " << gps_data_buff.at(i).local_position_ned.transpose() << std::endl;
        std::cout << "velocity: " << gps_data_buff.at(i).velocity.transpose() << std::endl;
        std::cout << "true velocity" << gps_data_buff.at(i).true_velocity.transpose() << std::endl;
    }

    return 0;
}

