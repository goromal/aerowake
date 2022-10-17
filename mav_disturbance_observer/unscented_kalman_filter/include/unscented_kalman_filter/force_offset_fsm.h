#ifndef UNSCENTED_KALMAN_FILTER_FORCE_OFFSET_FSM_H
#define UNSCENTED_KALMAN_FILTER_FORCE_OFFSET_FSM_H

#include <stdio.h>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>

// back-end
#include <boost/msm/back/state_machine.hpp>
//front-end
#include <boost/msm/front/state_machine_def.hpp>

namespace force_offset_fsm {

namespace msm_front = boost::msm::front;
namespace msm = boost::msm;
namespace mpl = boost::mpl;

class ForceOffsetFsmDef; // fwd definition
typedef boost::msm::back::state_machine<ForceOffsetFsmDef> ForceOffsetFsm;

// SM definition structure
class ForceOffsetFsmDef : public msm::front::state_machine_def<ForceOffsetFsmDef> {
 private:
  // event, best if defined outside the class
  struct ComputeAndUseNewOffset {};
  struct LoadFromParams {};
  struct TimerFired {};

  // states
  struct Computed;
  struct FromParams;
  struct Computing;

  // Actions
  // currently no actions

 public:
  // define initial state
  typedef FromParams initial_state;

  // define transition table
  typedef ForceOffsetFsmDef Fsm;
  struct transition_table : mpl::vector <
  //     Start          Event                     Next       Action                            Guard
  //     +------------+-------------------------+-----------+------------------------------+---------+
    _row < FromParams, ComputeAndUseNewOffset,  Computing  >,
    _row < FromParams, LoadFromParams,          FromParams >,
    _row < Computing,  TimerFired,              Computed   >,
    _row < Computing,  ComputeAndUseNewOffset,  Computing  >,
    _row < Computing,  LoadFromParams,          FromParams >,
    _row < Computed,   ComputeAndUseNewOffset,  Computing  >,
    _row < Computed,   LoadFromParams,          FromParams >
    > {};

  // No-transition response.
  template <class FSM, class Event>
  void no_transition(Event const& e, FSM&, int state) {
    std::cout << "no transition from state " << state
              << " on event " << typeid(e).name() << std::endl;
  }

  // class methods and member variables
 public:
  ForceOffsetFsmDef(const ros::NodeHandle nh_private);
  ~ForceOffsetFsmDef() {};
  void useParamOffset();
  void updateTimerDuration(const double timer_duration);
  void updateParamOffset(const Eigen::Vector3d &offset);
  void computeAndUseNewOffset();
  void getCurrentOffset(const Eigen::Vector3d &estimated_force, Eigen::Vector3d *offset);

 private:
  ros::NodeHandle nh_private_;
  ros::Timer timer_;

  double timer_duration_;
  Eigen::Vector3d current_offset_, param_offset_, integrated_offset_;
  std::size_t integration_step_;
  bool offset_integration_active_;

  void Initialize(const ros::NodeHandle nh_private);
  void TimerCallback(const ros::TimerEvent& event);

  // definition of fsm states
  struct FromParams : public msm_front::state<> {
    template<class Event, class FSM>
    void on_entry(Event const & evt, FSM & fsm) {
      fsm.current_offset_ = fsm.param_offset_;
      ROS_INFO("Unscented kalman filter: using force offset loaded value: [%.2f, %.2f, %.2f] [N].",
               fsm.param_offset_(0), fsm.param_offset_(1), fsm.param_offset_(2));
    }
  };

  struct Computing : public msm_front::state<> {
    template<class Event, class FSM>
    void on_entry(Event const & evt, FSM & fsm) {
      fsm.integration_step_ = 0;
      fsm.integrated_offset_.setZero();
      fsm.offset_integration_active_ = true;
      ros::Duration d(fsm.timer_duration_);
      fsm.timer_.setPeriod(d);
      fsm.timer_.start();
      ROS_INFO("Unscented kalman filter: started force offset computation (time: %.2f [s])", fsm.timer_duration_);

    }
    template<class Event, class FSM>
    void on_exit(Event const & evt, FSM & fsm) {
      fsm.timer_.stop();
      fsm.offset_integration_active_ = false;
    }
  };

  struct Computed : public msm_front::state<> {
    template<class Event, class FSM>
    void on_entry(Event const & evt, FSM & fsm) {
      if (fsm.integration_step_ != 0) {
        fsm.current_offset_ = fsm.integrated_offset_ / fsm.integration_step_;

      } else {
        ROS_FATAL("Unscented kalman filter: something went wrong in computing the force offset.");
      }
      fsm.integration_step_ = 0;
      ROS_INFO("Unscented kalman filter: force offset computed: [%.2f, %.2f, %.2f] [N]", fsm.current_offset_(0), fsm.current_offset_(1), fsm.current_offset_(2));
    }
    template<class Event, class FSM>
    void on_exit(Event const & evt, FSM & fsm) {
    }
  };


};
} // namespace force_offset_fsm
#endif // UNSCENTED_KALMAN_FILTER_FORCE_OFFSET_FSM_H

/*
namespace force_offset_fsm {

class ForceOffsetFsmDef {

 public:
  ForceOffsetFsm(const ros::NodeHandle nh_private);
  ~ForceOffsetFsm() {};
  void updateParamOffset(const Eigen::Vector3d &offset); //unused
  void computeAndUseNewOffset(); // ok
  void useParamOffset(); // ok
  void getCurrentOffset(const Eigen::Vector3d &estimated_force, Eigen::Vector3d *offset);
  void TimerCallback(const ros::TimerEvent& event);
  void updateTimerDuration(const double timer_duration); // ok

 private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ros::NodeHandle nh_private_; ok

  Eigen::Vector3d current_offset_, param_offset_, integrated_offset_; ok

  enum State {FROM_PARAM, COMPUTING, COMPUTED}; no
  State current_state_; no
  enum Input {COMPUTE_NEW, USE_PARAM, NONE, TIMER_FIRED}; no
  Input current_input_; no

  double timer_duration_; ok
  ros::Timer timer_; ok

  std::size_t integration_step_; ok

  void Initialize(const ros::NodeHandle nh_private); ok
};
*/