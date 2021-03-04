//
// Created by meng on 2021/3/1.
//

#ifndef GPS_IMU_FUSION_OBSERVABILITY_ANALYSIS_H
#define GPS_IMU_FUSION_OBSERVABILITY_ANALYSIS_H

#include <eigen3/Eigen/Dense>

#include <vector>

class ObservabilityAnalysis{
public:
    typedef typename Eigen::Matrix<double, 15, 15> TypeF;
    typedef typename Eigen::Matrix<double, 3, 15> TypeG;
    typedef typename Eigen::Matrix<double, 3, 1> TypeY;

    ObservabilityAnalysis() = default;

    bool ComputeSOM(const int &interval = 100, const int& number = 1);

    bool ComputeObservability();

    bool SaveFG(const TypeF &F, const TypeG &G, const TypeY &Y, const double& time);

private:
    struct FG
    {
        double time;
        Eigen::Matrix<double, 15, 15> F;
        Eigen::Matrix<double, 3, 15> G;
        std::vector<Eigen::Matrix<double, 3, 1>> Y;
    };

    std::vector<FG> FGs_;

    Eigen::MatrixXd Qso_;
    Eigen::MatrixXd Ys_;

    std::vector<TypeF> F_buffer_;
    std::vector<TypeG> G_buffer_;
    std::vector<TypeY> Y_buffer_;
};
#endif //GPS_IMU_FUSION_OBSERVABILITY_ANALYSIS_H
