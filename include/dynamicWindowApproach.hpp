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

        std::vector<Eigen::VectorXd> rolloutTrajectories();

        double computeDistanceToGoalCost(std::vector<Eigen::VectorXd> trajectory);

        double computeDistanceToObstacleCost(std::vector<Eigen::VectorXd> trajectory);

        double computeVelocityCost(std::vector<Eigen::VectorXd> trajectory);

        void predictState(Eigen::Vector2d &control, Eigen::VectorXd &state);  

        std::vector<Eigen::VectorXd> calculateTrajectory(Eigen::Vector2d &control);  

        void dynamicWindow();

        void globalPath();

        cv::Point2i cv_offset(float x, float y, int image_width, int image_height);

        void run();


    private:
        double pi_ = 3.14159;
        double maxLinearVelocity_ = 2;
        double minLinearVelocity_ = -0.5;
        double maxAngularVelocity_ = 40 * pi_/180;
        double maxLinearAcceleration_ = 0.2;
        double maxAngularAcceleration_ = 40 * pi_/180;
        double dt_ = 0.1;
        double windowTime_ = 3;
        double velocityResolution_ = 0.01;
        double angularVelocityResolution_ = 0.1 * pi_/180;
        double goalCostFactor_ = 1;
        double globalpathCostFactor_ = 1;
        double obstacleCostFactor_ = 1;
        double robotRadius_ = 1;
        int stateSize_ = 5;
        Eigen::MatrixXd transferFunction_;
        Eigen::VectorXd state_;
        Eigen::VectorXd currentState_;
        Eigen::Vector2d control_;
        std::vector<std::string> stateNames;
        std::vector<std::vector<double>> obstacles_ = {{-1, -1}, {0, 2}, {4.0, 2.0}, {5.0, 4.0}, {5.0, 5.0}, {5.0, 6.0}, {5.0, 9.0}, {8.0, 9.0}, {7.0, 9.0}, {12.0, 12.0}};
        std::vector<double> currentWindow_;
        Eigen::Vector2d goal_;
};

#endif  //  INCLUDE_DYNAMICWINDOWAPPROACH_HPP_