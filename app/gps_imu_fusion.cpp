//
// Created by meng on 2021/2/24.
//
#include "eskf_flow.h"
#include "global_defination.h"

int main(int argc, char** argv){

    std::string work_space_path = WORK_SPACE_PATH;
    ESKFFlow eskf_flow(work_space_path);

    //使用该函数时相当于只使用IMU位姿解算
    //eskf_flow.TestRun();//only predict

    eskf_flow.Run();

    return 0;
}
