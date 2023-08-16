//
// Created by Zhang Zhimeng on 23-7-16.
//
#include <iostream>
#include "common_tool.h"

int main(int argc, char **argv) {
    Eigen::Vector3d V_0_Ned(1, 2, 3);
    Eigen::Vector3d V_0_Enu = NedToEnu(V_0_Ned);

    std::cout << "Test 0" << std::endl;
    std::cout << "V_0_Ned: " << V_0_Ned.transpose() << std::endl;
    std::cout << "V_0_Enu: " << V_0_Enu.transpose() << std::endl << std::endl;

    Eigen::Vector3d V_1_Ned(0, 0, 0);
    Eigen::Vector3d V_1_Enu = NedToEnu(V_1_Ned);

    std::cout << "Test 1" << std::endl;
    std::cout << "V_1_Ned: " << V_1_Ned.transpose() << std::endl;
    std::cout << "V_1_Enu: " << V_1_Enu.transpose() << std::endl << std::endl;

    Eigen::Vector3d V_2_Ned(0, 0.1, 100);
    Eigen::Vector3d V_2_Enu = NedToEnu(V_2_Ned);

    std::cout << "Test 2" << std::endl;
    std::cout << "V_2_Ned: " << V_2_Ned.transpose() << std::endl;
    std::cout << "V_2_Enu: " << V_2_Enu.transpose() << std::endl;

    return 0;
}
