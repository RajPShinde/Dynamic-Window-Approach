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
        DynamicWindowApproach(double x, double y);

        /**
        *   @brief Destructor of DynamicWindowApproach
        *   @param none
        *   @return none
        */
        ~DynamicWindowApproach();

        void rolloutTrajectories();

        void computeCost();

        Eigen::VectorXd predictState(Eigen::Vector2d control_, Eigen::VectorXd state_);    

        void dynamicWindow();

        void globalPath();    

        void run();


    private:
        double maxLinearVelocity_ = 2;
        double minLinearVelocity_ = -1;
        double maxAngularVelocity_ = 0.5;
        double maxLinearAcceleration_ = 0.2;
        double maxAngularAcceleration_ = 0.1;
        double timeStep_ = 0.01;
        double windowTime_ = 3;
        double velocityResolution = 0.05;
        double angularVelocityResolution = 0.05;
        double goalCostFactor_;
        double globalpathCostFactor_;
        double obstacleCostFactor_;
        Eigen::MatrixXd transferFunction_;
        Eigen::VectorXd state_;
        Eigen::VectorXd currentState_
        Eigen::Vector2d control_;
        Eigen::MatrixXd transferFunction_;
        std::vector<std::string> stateNames;
        Eigen::Vector2d goal_;
        std::vector<double> obstacles_ = {};


};

#endif  //  INCLUDE_DYNAMICWINDOWAPPROACH_HPP_