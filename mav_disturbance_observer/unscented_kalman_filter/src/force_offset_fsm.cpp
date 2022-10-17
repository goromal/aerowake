# include "unscented_kalman_filter/force_offset_fsm.h"

namespace force_offset_fsm {

ForceOffsetFsmDef::ForceOffsetFsmDef(const ros::NodeHandle nh_private)
	: integration_step_(0),
	  integrated_offset_(0.0, 0.0, 0.0) {
	this->Initialize(nh_private);
};

void ForceOffsetFsmDef::Initialize(const ros::NodeHandle nh_private) {

	ROS_INFO("Unscented kalman filter: initializing force offset finite state machine.\n");

	nh_private_ = nh_private;

	double x_w, y_w, z_w;
	if (!nh_private_.getParam("force_offset/x", x_w)) {
		ROS_WARN("Unscented kalman filter: unable to get \'force_offset/x\'. Using 0.0 [N].");
		x_w = 0.0;
	}
	if (!nh_private_.getParam("force_offset/y", y_w)) {
		ROS_WARN("Unscented kalman filter: unable to get \'force_offset/y\'. Using 0.0 [N].");
		y_w = 0.0;
	}
	if (!nh_private_.getParam("force_offset/z", z_w)) {
		ROS_WARN("Unscented kalman filter: unable to get \'force_offset/z\'. Using 0.0 [N].");
		z_w = 0.0;
	}
	param_offset_ << x_w, y_w, z_w;

	if (!nh_private_.getParam("force_offset_computation_time", timer_duration_)) {
		ROS_WARN("Unscented kalman filter: unable to get \'offset_computation_time\'. Using 2.0 [s].");
		timer_duration_ = 2.0;
	}

	timer_ = nh_private_.createTimer(ros::Duration(timer_duration_),
	                                 &ForceOffsetFsmDef::TimerCallback, this, false,
	                                 false);
	offset_integration_active_ = false; // true while offset is computed
	current_offset_ = param_offset_;
	ROS_INFO("Unscented kalman filter: force offset finite state machine initialized.\n");
}

void ForceOffsetFsmDef::updateParamOffset(const Eigen::Vector3d & offset) {
	param_offset_ = offset;
}

void ForceOffsetFsmDef::updateTimerDuration(const double timer_duration) {
	timer_duration_ = timer_duration;
}


void ForceOffsetFsmDef::computeAndUseNewOffset() {
	msm::back::state_machine<ForceOffsetFsmDef> &fsm = static_cast<msm::back::state_machine<ForceOffsetFsmDef> &>(*this);
  	fsm.process_event(ComputeAndUseNewOffset());
}

void ForceOffsetFsmDef::useParamOffset() {
	msm::back::state_machine<ForceOffsetFsmDef> &fsm = static_cast<msm::back::state_machine<ForceOffsetFsmDef> &>(*this);
  	fsm.process_event(LoadFromParams());
}

void ForceOffsetFsmDef::TimerCallback(const ros::TimerEvent & event) {
	msm::back::state_machine<ForceOffsetFsmDef> &fsm = static_cast<msm::back::state_machine<ForceOffsetFsmDef> &>(*this);
  	fsm.process_event(TimerFired());
}

/*
void ForceOffsetFsmDef::StartTimerActionCallback(ComputeAndUseNewOffset const&) {
	//integrated_offset_.setZero();
	//timer_.stop();
	//ros::Duration d(timer_duration_);
	//timer_.setPeriod(d);
	//timer_.start();
	//offset_integration_active_ = true;
	//ROS_INFO("Unscented kalman filter: started force offset computation (time: %.2f [s])", timer_duration_);
}

void ForceOffsetFsmDef::LoadFromParamsActionCallback(LoadFromParams const&) {
	timer_.stop();
	offset_integration_active_ = false;
}

void ForceOffsetFsmDef::ApplyComputedOffsetActionCallback(TimerFired const&) {
	offset_integration_active_ = false;
	if (integration_step_ != 0) {
		current_offset_ = integrated_offset_ / integration_step_;

	} else {
		ROS_FATAL("Unscented kalman filter: something went wrong in computing the force offset.");
	}
	integrated_offset_.setZero();
	integration_step_ = 0;
	ROS_INFO("Unscented kalman filter: force offset computed: [%.2f, %.2f, %.2f] [N]", current_offset_(0), current_offset_(1), current_offset_(2));
}
*/

void ForceOffsetFsmDef::getCurrentOffset(const Eigen::Vector3d & estimated_force, Eigen::Vector3d *offset) {

	msm::back::state_machine<ForceOffsetFsmDef> &fsm = static_cast<msm::back::state_machine<ForceOffsetFsmDef> &>(*this);
	// in state actions
	if (offset_integration_active_) {
		integration_step_++;
		integrated_offset_ += estimated_force;
	}

	*offset = current_offset_;
}
} // end force_offset_fsm