#ifndef _PROBABILISTIC_TRAJECTORY_PREDICTION_
#define _PROBABILISTIC_TRAJECTORY_PREDICTION_

#include <vector>
#include <iostream>
#include <Eigen/Dense>
using Eigen::MatrixXd;

class ProbabilisticTrajectoryPrediction{
    private:
        const double delta_t = 0.01;        // 10 ms
        const double delta_t_max = 0.16;    // 160 ms
        const double max_time = 3.0;        // 3 sec
    public:
        std::vector<std::vector<double>> mu_sigmas_;
        ProbabilisticTrajectoryPrediction();

        const std::vector<std::vector<double>>& getMuSigma(double x, double y, double h,
                                                     double sigma_xx, double sigma_yy,
                                                     double sigma_xy);
        void updateABasedOnInterval(MatrixXd& A, double interval);
        void pushBackMuSigma(std::vector<double>& row, MatrixXd& mu, MatrixXd& covariance);
        void forward_predict(MatrixXd& mu, MatrixXd& covariance, MatrixXd& A, MatrixXd& Q);
};

#endif