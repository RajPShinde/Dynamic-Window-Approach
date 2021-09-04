#include <dynamicWindowApproach.hpp>

DynamicWindowApproach::DynamicWindowApproach(double x, double y): goal_(x, y) {
	std::cout<<"DWA\n";
	state_.resize(stateSize_);
	state_.setZero();
	currentState_.resize(stateSize_);
	currentState_.setZero();
	transferFunction_.resize(stateSize_,stateSize_);
	transferFunction_.setIdentity();

	run();
}

DynamicWindowApproach::~DynamicWindowApproach() {
}

void DynamicWindowApproach::predictState(Eigen::Vector2d &control, Eigen::VectorXd &state){

	// transferFunction_(0, 3) = std::cos(state(2)) * dt_;
	// transferFunction_(1, 3) = std::sin(state(2)) * dt_;
	// transferFunction_(2, 2) = dt_;
	// state(3) = control(0);
	// state(4) = control(1);
	// state = transferFunction_ * state;
	state(2) += control(1) * dt_;
	state(0) += control(0) * std::cos(state(2)) * dt_;
	state(1) += control(0) * std::sin(state(2)) * dt_;
	state(3) = control(0);
	state(4) = control(1);
}

void DynamicWindowApproach::dynamicWindow(){
	currentWindow_.clear();
	double currLinearVelocity = currentState_(3);
	double currTheta = currentState_(4);
 	
 	// Velocity and Theta Increment with maximum Linear and angular Acceleration
 	double linearVelocityIncrement = maxLinearAcceleration_ * dt_;
 	double angularVelocityIncrement = maxAngularAcceleration_ * dt_;

 	// Below linear velocity range will contain all possible linear velocities produced by diffrent linear accelerations constrained by maximum linear velocity and acceleration
	// Min possible velocity in dt
	currentWindow_.push_back(std::max(currLinearVelocity - linearVelocityIncrement, minLinearVelocity_));
	// Max possible velocity in dt
	currentWindow_.push_back(std::min(currLinearVelocity + linearVelocityIncrement, maxLinearVelocity_));


	// Below angular velocity range will contain all possible angular velocities produced by diffrent angular accelerations constrained by maximum angular velocity and acceleration
	// Min possible theta in dt
	currentWindow_.push_back(std::max(currTheta - angularVelocityIncrement, -maxAngularVelocity_));
	// Max possible theta in dt
	currentWindow_.push_back(std::min(currTheta + angularVelocityIncrement, maxAngularVelocity_));
}

std::vector<Eigen::VectorXd> DynamicWindowApproach::calculateTrajectory(Eigen::Vector2d &control){
	std::vector<Eigen::VectorXd> trajectory;
	double time = 0;
	int i =0;
	Eigen::VectorXd state = currentState_;
	trajectory.push_back(state);
	while(time <= windowTime_){
		predictState(control, state);
		trajectory.push_back(state);
		
		time = time + dt_;
		i++;
	}
	return trajectory;
}

std::vector<Eigen::VectorXd> DynamicWindowApproach::rolloutTrajectories(){
	double cost = 0;
	double minCost = INT_MAX;
	std::vector<Eigen::VectorXd> bestTrajectory;
	// Find all trajectories with sampled input in dynamic window
	for(double i = currentWindow_[0]; i<=currentWindow_[1]; i = i + velocityResolution_){
		for(double j = currentWindow_[2]; j<=currentWindow_[3]; j = j + angularVelocityResolution_){
			Eigen::Vector2d control(i, j);
			std::vector<Eigen::VectorXd> trajectory;
			trajectory = calculateTrajectory(control);

			// Compute cost for the trajectory
			cost = computeDistanceToGoalCost(trajectory) + computeDistanceToObstacleCost(trajectory) + computeVelocityCost(trajectory);
			
			if(cost < minCost){
				minCost = cost;
				bestTrajectory = trajectory;
			}
		}
	}
	return bestTrajectory;
}


double DynamicWindowApproach::computeDistanceToGoalCost(std::vector<Eigen::VectorXd> trajectory){
	double goalCost;

	// Cost: Distance to goal (using error Angle)
	double startGoalVector = std::sqrt(goal_(0)*goal_(0) + goal_(1)*goal_(1));
	double startCurrentVector = std::sqrt(std::pow(trajectory.back()(0), 2) + std::pow(trajectory.back()(1), 2));
	double dotProduct = (goal_(0) * trajectory.back()(0))+ (goal_(1) * trajectory.back()(1));
	double cosTheta = dotProduct / (startGoalVector * startCurrentVector);
	double theta = std::acos(cosTheta);
	goalCost = goalCostFactor_ * theta;

	// double dx1 = trajectory.back()(0) - goal_(0);
	// double dy1 = trajectory.back()(1) - goal_(1);
	// double dx2 = goal_(0);
	// double dy2 = goal_(1);
	// double cross = abs(dx1*dy2 - dx2*dy1);
	// goalCost = cross*0.001;

	return goalCost;
}

double DynamicWindowApproach::computeDistanceToObstacleCost(std::vector<Eigen::VectorXd> trajectory){
	double obstacleCost = INT_MAX;
	double minimumSeparation = INT_MAX;

	// Cost: Distance to Obstacle
	for (int i=0; i<trajectory.size(); i=i+2){
    	for (int j=0; j< obstacles_.size(); j++){
      		double oX = obstacles_[j][0];
      		double oY = obstacles_[j][1];
      		double dx = trajectory[i][0] - oX;
      		double dy = trajectory[i][1] - oY;

      		double separation = std::sqrt(dx*dx + dy*dy);
      		if (separation <= robotRadius_){
          		return obstacleCost;
      		}

      		if (minimumSeparation >= separation){
          		minimumSeparation = separation;
      		}
    	}
  	}
  	obstacleCost = 1 / minimumSeparation;

	return obstacleCost;
}

double DynamicWindowApproach::computeVelocityCost(std::vector<Eigen::VectorXd> trajectory){
	double velocityCost= maxLinearVelocity_ - trajectory.back()[3];
	return velocityCost;
}

void DynamicWindowApproach::globalPath(){
}

cv::Point2i DynamicWindowApproach::cv_offset(
    float x, float y, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + image_width/2;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
};


void DynamicWindowApproach::run(){

	cv::namedWindow("Dynamic Window Approach: Motion Planner", cv::WINDOW_NORMAL);
	bool goalNotReached = true;
	std::vector<Eigen::VectorXd> bestTrajectory;

	// Initial x, y, theta, velovity, angularVelocity
	currentState_<< 0,0,3.14/8,0,0;

	// Goal
	goal_<< 10, 10;

	// Run Motion Planner till robot reaches Goal
	while(goalNotReached){

		// find dynamic window
		dynamicWindow();

		// find Trajectories as well as find best of all those based on cost functions
		bestTrajectory = rolloutTrajectories();

		// Send the control commond to robot that leads to bestTrajectory for time dt
		control_(0) = bestTrajectory[1](3);
		control_(1) = bestTrajectory[1](4);
		std::cout<<"\033[1m\033[32mLinear Velocity-\033[0m"<<control_(0)<<" \033[1m\033[32mAngular Velocity-\033[0m"<<control_(1)<<"\n";
		predictState(control_, currentState_);

		// Draw 
		// Window
		cv::Mat dwa(3500,3500, CV_8UC3, cv::Scalar(255,255,255));
		// Goal
		cv::circle(dwa, cv_offset(goal_(0), goal_(1), dwa.cols, dwa.rows),30, cv::Scalar(0,255,0), 5);
		// Robot Location
		cv::circle(dwa, cv_offset(currentState_(0), currentState_(1), dwa.cols, dwa.rows), 30, cv::Scalar(0,0,0), 5);
		// Robot Orientation
		cv::arrowedLine(dwa, cv_offset(currentState_(0), currentState_(1), dwa.cols, dwa.rows), 
							cv_offset(currentState_(0) + std::cos(currentState_(2)), currentState_(1) 
									  + std::sin(currentState_(2)), dwa.cols, dwa.rows), cv::Scalar(0,0,0), 7);
		// Obstacles
		for(int i = 0; i<obstacles_.size(); i++){
			cv::circle(dwa, cv_offset(obstacles_[i][0], obstacles_[i][1], dwa.cols, dwa.rows), 20, cv::Scalar(0,0,0), -1);
		}
		// Best Trajectory
		for(int i = 0; i<bestTrajectory.size(); i++){
			cv::circle(dwa, cv_offset(bestTrajectory[i](0), bestTrajectory[i](1), dwa.cols, dwa.rows), 7, cv::Scalar(0,0,255), -1);
		}

		// Check if robot is within goal threshold radius
		if(std::sqrt(std::pow(currentState_(0) - goal_(0), 2) + std::pow(currentState_(1) - goal_(1), 2) <= robotRadius_)){
			goalNotReached = false;
			cv::waitKey(0);
		}

    cv::imshow("Dynamic Window Approach: Motion Planner", dwa);
    cv::waitKey(5);
	}

}

       
