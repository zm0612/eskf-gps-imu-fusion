//
// Created by meng on 2021/2/24.
//
#include "eskf_flow.h"

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_alsologtostderr = true;

    if (argc != 3) {
        std::cout << "Please enter right command and parameters!" << std::endl;

        return -1;
    }

    std::string config_file_path(argv[1]);
    std::string data_file_path(argv[2]);

    ESKFFlow eskf_flow(config_file_path, data_file_path);

    //使用该函数时相当于只使用IMU位姿解算
//    eskf_flow.TestRun();//only predict

    eskf_flow.Run();

    return 0;
}
