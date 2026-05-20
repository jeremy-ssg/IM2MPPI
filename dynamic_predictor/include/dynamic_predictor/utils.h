#ifndef DYNAMIC_PREDICTOR_UTILS_H
#define DYNAMIC_PREDICTOR_UTILS_H

#include<Eigen/Eigen>
namespace dynamicPredictor{
    struct obstacle
    {
        /* data */
        // std::vector<Eigen::Vector3d> posHist;
        std::vector<std::vector<Eigen::Vector3d>> posPred;
        std::vector<std::vector<Eigen::Vector3d>> sizePred;
        // Per-(intent, step) empirical standard deviation of predicted
        // obstacle position, derived from the spread of allPredPoints_ samples
        // inside the predictor. Used by IM2-MPPI's CVaR risk term so that the
        // Monte-Carlo uncertainty sampling tracks the predictor's actual
        // confidence rather than a hard-coded constant.
        std::vector<std::vector<Eigen::Vector3d>> sigmaPred;
        Eigen::VectorXd intentProb; // front, left, right, stop
    };

    enum intentType{
        FORWARD,
        LEFT,
        RIGHT,
        STOP
    };
    
    
}
#endif