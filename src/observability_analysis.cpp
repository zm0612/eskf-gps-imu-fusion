//
// Created by meng on 2021/3/1.
//
#include "observability_analysis.h"

#include <iostream>

bool ObservabilityAnalysis::ComputeSOM(const int &interval, const int &number) {
    Qso_.resize(3*15*FGs_.size(), 15);
    std::cout << Qso_.rows() << " : " << Qso_.cols() << std::endl;
    Ys_.resize(3*15*FGs_.size(), 1);
    std::cout << Ys_.rows() << " : " << Ys_.cols() << std::endl;
    Eigen::Matrix<double, 15, 15> Fnn = Eigen::Matrix<double, 15, 15>::Identity();
    for (int i = 0; i < FGs_.size(); ++i)
    {
        Eigen::Matrix<double, 15, 15> Fn = Eigen::Matrix<double, 15, 15>::Identity();
        for (int j = 0; j < 15; j++)
        {
            if (j > 0)
            {
                Fn = Fn * FGs_[i].F;
            }
            Ys_.block<3, 1>(i*3*15+3*j, 0) = FGs_[i].Y[j];
            Qso_.block<3, 15>(i*3*15+3*j, 0) = FGs_[i].G * Fn * Fnn;
        }
    }
    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(Qso_);
    auto rank = lu_decomp.rank();
    std::cout << "matrix rank: " << rank << std::endl;

    return true;
}

bool ObservabilityAnalysis::ComputeObservability() {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Qso_, Eigen::ComputeFullU | Eigen::ComputeThinV);

    for (int i = 0; i < 15; ++i) {
        double temp = (svd.matrixU().row(i) * Ys_)[0] / svd.singularValues()[i];
        Eigen::MatrixXd Xi = temp * svd.matrixV().col(i);
        Eigen::MatrixXd::Index max_row, max_col;
        Xi = Xi.cwiseAbs();
        double max_value = Xi.maxCoeff(&max_row, &max_col);
        std::cout << svd.singularValues()[i] / svd.singularValues()[0] << "," << max_row << std::endl;
    }

    return true;
}

bool ObservabilityAnalysis::SaveFG(const TypeF &F, const TypeG &G, const TypeY &Y, const double& time) {
    static int FGSize = 10;
    static double time_interval = 100;

    if (FGs_.size() > FGSize)
    {
        return true;
    }
    if (FGs_.empty())
    {
        FG fg;
        fg.time = time;
        // fg.F = Ft;
        fg.F = F - Eigen::Matrix<double, 15, 15>::Identity();
        // fg.F = (Ft - Eigen::Matrix<double, 15, 15>::Identity()) / T;
        fg.G = G;
        fg.Y.push_back(Y);
        FGs_.push_back(fg);
    }
    else
    {
        if (FGs_.back().Y.size() == 15)
        {
            if (time - FGs_.back().time < time_interval || FGs_.size() >= FGSize)
            {
                return true;
            }
            FG fg;
            fg.time = time;
            // fg.F = Ft;
            fg.F = F - Eigen::Matrix<double, 15, 15>::Identity();
            // fg.F = (Ft - Eigen::Matrix<double, 15, 15>::Identity()) / T;
            fg.G = G;
            fg.Y.push_back(Y);
            FGs_.push_back(fg);
        }
        else
        {
            FGs_.back().Y.push_back(Y);
        }

    }
    return true;
}