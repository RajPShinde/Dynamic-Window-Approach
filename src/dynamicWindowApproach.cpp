#include <dynamicWindowApproach.hpp>

DynamicWindowApproach::DynamicWindowApproach() {
	transferFunction_.resize(stateSize,stateSize);
	transferFunction_.setIdentity();
}

DynamicWindowApproach::~DynamicWindowApproach() {
}

void DynamicWindowApproach::predictState(Eigen::Vector2d control_, Eigen::VectorXd state_){
	double theta = state_(2);
	
	transferFunction_(x, linearVelocity) = control_(0) * cos(theta) * dt;
	transferFunction_(y, linearVelocity) = control_(0) * sin(theta) * dt;
	transferFunction_(theta, theta) = dt;

	state_(3) = control_(0);
	state_(4) = control_(1);

	state_ = transferFunction_ * state_;
}

void DynamicWindowApproach::rolloutTrajectories(){

}

void DynamicWindowApproach::computeCost(){

}

void DynamicWindowApproach::run(){

}

       
