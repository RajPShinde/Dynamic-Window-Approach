#include <dynamicWindowApproach.hpp>

DynamicWindowApproach::DynamicWindowApproach() {
	transferFunction_.resize(stateSize,stateSize);
	transferFunction_.setIdentity();
}

DynamicWindowApproach::~DynamicWindowApproach() {
}

Eigen::VectorXd DynamicWindowApproach::predictState(Eigen::Vector2d &control_, Eigen::VectorXd state_){
	double theta = state_(2);

	transferFunction_(x, linearVelocity) = control_(0) * cos(theta) * dt;
	transferFunction_(y, linearVelocity) = control_(0) * sin(theta) * dt;
	transferFunction_(theta, theta) = dt;

	state_(3) = control_(0);
	state_(4) = control_(1);

	state_ = transferFunction_ * state_;

	retrun state_
}

void DynamicWindowApproach::dynamicWindow(Eigen::VectorXd &currentState_){
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

void DynamicWindowApproach::rolloutTrajectories(){
	
	// Find all trajectories with sampled input in dynamic window
	for(int i = currentWindow_(0); i<=currentWindow_(1); i = i + velocityResolution){
		for(int j = currentWindow_(2); j<=currentWindow_(3); j = j + angularVelocityResolution){
			calulateTrajectory(i, j);
		}
	}

}


void DynamicWindowApproach::computeCost(){

}

void DynamicWindowApproach::globalPath(){
}

void DynamicWindowApproach::run(){

	globalPath = globalPath();
	std::vector<double> obstacles = {};
	bool goalReached = false;
	// Initial x, y, theta, velovity, angularVelocity
	currentState_<< 0,0,0,0,0;  
	
	// Run Motion Planner till robot reaches Goal
	while(goalReached){

		// find dynamic window
		dynamicWindow(currentState_);
		// find Trajectories
		trajectories = rolloutTrajectories();
		// find best trajectory based on cost
		bestTrajectory(globalPath, obstacles, trajectories, currentState_);
		// Send the control commond to robot that leads to bestTrajectory for time dt
		predictState(currentState_, control);
		// Check if robot is within goal threshold radius
		if(){
			goalReached = true;
		}

	}

}

       
