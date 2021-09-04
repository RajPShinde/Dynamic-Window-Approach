#include <dynamicWindowApproach.hpp>

DynamicWindowApproach::DynamicWindowApproach(double x, double y): goal_(x, y) {
	state_.resize(stateSize);
	state_.setzero();
	currentState_.resize(stateSize);
	current_.setzero();
	transferFunction_.resize(stateSize,stateSize);
	transferFunction_.setIdentity();

	run();
}

DynamicWindowApproach::~DynamicWindowApproach() {
}

Eigen::VectorXd DynamicWindowApproach::predictState(Eigen::Vector2d &control, Eigen::VectorXd &state){
	double theta = state_(2);

	transferFunction_(x, linearVelocity) = control_(0) * cos(theta) * dt;
	transferFunction_(y, linearVelocity) = control_(0) * sin(theta) * dt;
	transferFunction_(theta, theta) = dt;

	state_(3) = control_(0);
	state_(4) = control_(1);

	state_ = transferFunction_ * state_;
}

void DynamicWindowApproach::dynamicWindow(){
	currentWindow_.clear();
	double currLinearVelocity = currentState_(3);
	double currTheta = currentState_(4);
 	
 	// Velocity and Theta Increment with maximum Linear and angular Acceleration
 	linearVelocityIncrement = maxLinearAcceleration_*dt;
 	angularVelocityIncrement = maxAngularAcceleration*dt;

 	// Below linear velocity range will contain all possible linear velocities produced by diffrent linear accelerations constrained by maximum linear velocity and acceleration
	// Min possible velocity in dt
	currentWindow_.push_back(std::max(currentState_(3) - linearVelocityIncrement, minLinearVelocity_));
	// Max possible velocity in dt
	currentWindow_.push_back(std::min(currentState_(3) + linearVelocityIncrement, maxLinearVelocity_));


	// Below angular velocity range will contain all possible angular velocities produced by diffrent angular accelerations constrained by maximum angular velocity and acceleration
	// Min possible theta in dt
	currentWindow_.push_back(std::max(currentState_(4) - angularVelocityIncrement, -maxAngularVelocity_));
	// Max possible theta in dt
	currentWindow_.push_back(std::min(currentState_(4) + angularVelocityIncrement, maxAngularVelocity_));
}

vod DynamicWindowApproach::calculateTrajectory(Eigen::Vector2d &control){
	std::vector<Eigen::Matrix<double, 5, 1>> trajectory;
	double time = 0;
	Eigen::VectorXd state = currentState_;
	while(time<=windowTime){
		predictState(control, state);
		trajectory.push_back(state);
		time = time + dt_;
	}
	return trajectory;
}

void DynamicWindowApproach::rolloutTrajectories(){
	double cost = 0;
	double minCost = INT_MAX;
	std::vector<Eigen::VectorXd> bestTrajectory;
	// Find all trajectories with sampled input in dynamic window
	for(int i = currentWindow_(0); i<=currentWindow_(1); i = i + velocityResolution_){
		for(int j = currentWindow_(2); j<=currentWindow_(3); j = j + angularVelocityResolution_){
			Eigen::Vector2d control << i, j;
			std::vector<Eigen::VectorXd> trajectory;
			trajectory = calulateTrajectory(control);

			// Compute cost for the trajectory
			cost = computeCost(trajectory)
			if(cost < minCost){
				minCost = cost;
				bestTrajectory = trajectory;
			}
		}
	}
	return bestTrajectory;
}


double DynamicWindowApproach::computeCost(std::vector<Eigen::VectorXd> trajectory){
	// Cost: Distance to goal

	// Cost: Distance to Obstacle

	// Cost: Distance from Global Path
	// Cost: Time to Goal
	// TO-DO
	return totalCost;
}

void DynamicWindowApproach::globalPath(){
}

void DynamicWindowApproach::run(){

	bool goalReached = false;
	// Initial x, y, theta, velovity, angularVelocity
	currentState_<< 0,0,0,0,0;  
	
	// Run Motion Planner till robot reaches Goal
	while(goalReached){
		// find dynamic window
		dynamicWindow();
		// find Trajectories as well as find best of all those based on cost functions
		bestTrajectory = rolloutTrajectories();
		// Send the control commond to robot that leads to bestTrajectory for time dt
		predictState(currentState_, control);

		// Draw

		// Check if robot is within goal threshold radius
		if(){
			goalReached = true;
		}

	}

}

       
