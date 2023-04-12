#include "probabilistic_trajectory_prediction.h"

ProbabilisticTrajectoryPrediction::ProbabilisticTrajectoryPrediction(){
    this->mu_sigmas_ = std::vector<std::vector<double>>(16, std::vector<double>());
}

void ProbabilisticTrajectoryPrediction::updateABasedOnInterval(MatrixXd& A, double interval){
    A << 1, interval, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, interval,
             0, 0, 0, 1;
}

void ProbabilisticTrajectoryPrediction::pushBackMuSigma(std::vector<double>& row, MatrixXd& mu, MatrixXd& covariance){
    row.push_back(mu(0));
    row.push_back(mu(2));
    row.push_back(covariance(0, 0));
    row.push_back(covariance(0, 2));
    row.push_back(covariance(2, 2));
}

const std::vector<std::vector<double>>& ProbabilisticTrajectoryPrediction::getMuSigma(
                                                     double x, double y, double h,
                                                     double sigma_xx, double sigma_yy,
                                                     double sigma_xy){
    
    double interval = 0.01;     // 10 ms
    MatrixXd mu(4, 1);
    MatrixXd covariance(4, 4);
    MatrixXd A(4, 4);
    MatrixXd Q(4, 4);

    const double velocity_x = 0.5;
    const double velocity_y = 0.5;
    
    mu << x, velocity_x, y, velocity_y;
    covariance << sigma_xx, 0, sigma_xy, 0,
                  0, 1, 0, 0,
                  sigma_xy, 0, sigma_yy, 0,
                  0, 0, 0, 1;
    Q = MatrixXd::Identity(4, 4);

    for(auto& row: mu_sigmas_){
        row.clear();
        updateABasedOnInterval(A, interval);
        for(double time = 0.0; time <= max_time - interval; time += 0.01){
            forward_predict(mu, covariance, A, Q);      // wrong logic
            pushBackMuSigma(row, mu, covariance);
        }
        interval += 0.01;
    }
    
    return mu_sigmas_;
}

void ProbabilisticTrajectoryPrediction::forward_predict(MatrixXd& mu, MatrixXd& covariance, MatrixXd& A, MatrixXd& Q){
    mu = A * mu;
    covariance = A*covariance*A.transpose() + Q;
}