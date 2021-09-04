#ifndef INCLUDE_DYNAMICWINDOWAPPROACH_HPP_
#define INCLUDE_DYNAMICWINDOWAPPROACH_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class DynamicWindowApproach
{
    public:

        /**
        *   @brief Constructor of DynamicWindowApproach
        *   @param none
        *   @return none
        */
        DynamicWindowApproach();

        /**
        *   @brief Destructor of DynamicWindowApproach
        *   @param none
        *   @return none
        */
        ~DynamicWindowApproach();

        void rolloutTrajectories();

        void computeCost();

        void predictState(Eigen::Vector2d control_, Eigen::VectorXd state_);        

        void run();


    private:
        double maxLinearVelocity_ = 2;
        double minLinearVelocity_ = -1;
        double angularVelocity_ = 0.5;
        double maxLinearAcceleration_ = 0.2;
        double AngularAcceleration_ = 0.1;
        double timeStep_ = 0.01;
        double windowTime_ = 3;
        double goalCostFactor_;
        double globalpathCostFactor_;
        double obstacleCostFactor_;
        Eigen::MatrixXd transferFunction_;
        Eigen::VectorXd state_;
        Eigen::Vector2d control_;
        Eigen::MatrixXd transferFunction_;
        std::vector<std::string> stateNames;


};

#endif  //  INCLUDE_DYNAMICWINDOWAPPROACH_HPP_