#include <iostream>
#include <Eigen/Dense>
#include "probabilistic_trajectory_prediction.h"
using Eigen::MatrixXd;
 
int main()
{
    ProbabilisticTrajectoryPrediction obj;
    const std::vector<std::vector<double>> ans = obj.getMuSigma(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    std::cout << "Row returned = " << ans.size() << std::endl;
    
    // MatrixXd m(2,2);
    // m(0,0) = 3;
    // m(1,0) = 2.5;
    // m(0,1) = -1;
    // m(1,1) = m(1,0) + m(0,1);
    // std::cout << m << std::endl;
}