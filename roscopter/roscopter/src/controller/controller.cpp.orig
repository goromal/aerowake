#include <controller/controller.h>
#include <stdio.h>

namespace controller
{

Controller::Controller() :
  nh_(ros::NodeHandle()),
  nh_private_("~")
{
  // Retrieve global MAV equilibrium throttle. This is the only MAV specific
  // parameter that is required
  ros::NodeHandle nh_mav(ros::this_node::getNamespace());
  if (!nh_private_.getParam("equilibrium_throttle", throttle_eq_))
    ROS_ERROR("[Controller] MAV equilibrium_throttle not found!");

  // Calculate max accelerations. Assuming that equilibrium throttle produces
  // 1 g of acceleration and a linear thrust model, these max acceleration
  // values are computed in g's as well.
  max_accel_z_ = 1.0 / throttle_eq_;
  max_accel_xy_ = sin(acos(throttle_eq_)) / throttle_eq_ / sqrt(2.);

  is_flying_ = false;
  armed_ = false;
  received_cmd_ = false;

  pn_0_ = 0.0;
  pe_0_ = 0.0;
  pd_0_ = 0.0;

  nh_private_.getParam("max_roll", max_.roll);
  nh_private_.getParam("max_pitch", max_.pitch);
  nh_private_.getParam("max_yaw_rate", max_.yaw_rate);
  nh_private_.getParam("max_throttle", max_.throttle);
  nh_private_.getParam("max_n_dot", max_.n_dot);
  nh_private_.getParam("max_e_dot", max_.e_dot);
  nh_private_.getParam("max_d_dot", max_.d_dot);

  nh_private_.getParam("min_altitude", min_altitude_);

  _func = boost::bind(&Controller::reconfigure_callback, this, _1, _2);
  _server.setCallback(_func);

  // Set up Publishers and Subscriber
  state_sub_ = nh_.subscribe("estimate", 1, &Controller::stateCallback, this);
  is_flying_sub_ =
      nh_.subscribe("is_flying", 1, &Controller::isFlyingCallback, this);
  cmd_sub_ =
      nh_.subscribe("high_level_command", 1, &Controller::cmdCallback, this);
  status_sub_ = nh_.subscribe("status", 1, &Controller::statusCallback, this);

  command_pub_ = nh_.advertise<rosflight_msgs::Command>("command", 1);
  full_command_pub_ = nh_.advertise<nav_msgs::Odometry>("full_hl_command", 1);
}


void Controller::stateCallback(const nav_msgs::OdometryConstPtr &msg)
{
  static double prev_time = 0;
  if(prev_time == 0)
  {
    prev_time = msg->header.stamp.toSec();
    return;
  }

  // Calculate time
  double now = msg->header.stamp.toSec();
  double dt = now - prev_time;
  prev_time = now;

  if(dt <= 0)
    return;

  // This should already be coming in NED
  xhat_.pn = msg->pose.pose.position.x;
  xhat_.pe = msg->pose.pose.position.y;
  xhat_.pd = msg->pose.pose.position.z;

  xhat_.u = msg->twist.twist.linear.x;
  xhat_.v = msg->twist.twist.linear.y;
  xhat_.w = msg->twist.twist.linear.z;

  // Convert Quaternion to RPY
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
  tf::Matrix3x3(tf_quat).getRPY(xhat_.phi, xhat_.theta, xhat_.psi);
  xhat_.theta = xhat_.theta;
  xhat_.psi = xhat_.psi;

  xhat_.p = msg->twist.twist.angular.x;
  xhat_.q = msg->twist.twist.angular.y;
  xhat_.r = msg->twist.twist.angular.z;

  if(/*is_flying_ && */armed_ && received_cmd_) // probably guards against the (throttle on ground) thing...
  {
    ROS_WARN_ONCE("CONTROLLER ACTIVE");
    computeControl(dt);
    publishCommand();
  }
  else
  {
    resetIntegrators();
    prev_time_ = msg->header.stamp.toSec();
  }
}


void Controller::isFlyingCallback(const std_msgs::BoolConstPtr &msg)
{
  is_flying_ = msg->data;
  if (is_flying_)
      std::cout << "WE'RE FLYING" << std::endl;
}

void Controller::statusCallback(const rosflight_msgs::StatusConstPtr &msg)
{
  armed_ = msg->armed;
  if (!armed_)
      received_cmd_ = false;
}


void Controller::cmdCallback(const rosflight_msgs::CommandConstPtr &msg)
{
//    armed_ = (msg->ignore != 8);
    if (!armed_)
    {
        command_.F = 0.0;
        command_.x = 0.0;
        command_.y = 0.0;
        command_.z = 0.0;
        command_.ignore = 8;
        command_.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
        publishCommand();
        received_cmd_ = false;
        return;
    }

  switch(msg->mode)
  {
    case rosflight_msgs::Command::MODE_PITCH_YVEL_YAW_ALTITUDE:
      if (control_mode_ == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
      {
          pd_0_ = xhat_.pd;
          ROS_INFO("New controller altitude offset: alt=%f", -pd_0_);
      }
      control_mode_ = static_cast<uint8_t>(msg->mode);
      xc_.pn = 0.0;
      xc_.pe = 0.0;
      xc_.pd = static_cast<double>(-msg->F) + pd_0_;
      // xc_.phi determined by control
      xc_.theta = static_cast<double>(msg->x);
      xc_.psi = static_cast<double>(msg->z);
      xc_.x_dot = 0.0;
      xc_.y_dot = static_cast<double>(msg->y);
      // xc_.z_dot determined by control
      // xc_.r determined by control
      break;
    case rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE:
      if (control_mode_ == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
      {
          pn_0_ = xhat_.pn;
          pe_0_ = xhat_.pe;
          pd_0_ = xhat_.pd;
          ROS_INFO("New positional offsets: pn=%f pe=%f pd=%f", pn_0_, pe_0_, -pd_0_);
      }
      control_mode_ = msg->mode;
      xc_.pn = static_cast<double>(msg->x)  + pn_0_;
      xc_.pe = static_cast<double>(msg->y)  + pe_0_;
      xc_.pd = static_cast<double>(-msg->F) + pd_0_;
      // xc_.phi determined by control
      // xc_.theta determined by control
      xc_.psi = static_cast<double>(msg->z);
      // xc_.x_dot determined by control
      // xc_.y_dot determined by control
      // xc_.z_dot determined by control
      // xc_.r determined by control
      break;
    case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
      if (control_mode_ == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
      {
          pd_0_ = xhat_.pd;
          ROS_INFO("New controller altitude offset: alt=%f", -pd_0_);
      }
      control_mode_ = msg->mode;
      xc_.pn = 0.0;
      xc_.pe = 0.0;
      xc_.pd = static_cast<double>(-msg->F) + pd_0_;
      // xc_.phi determined by control
      // xc_.theta determined by control
      xc_.psi = 0.0;
      xc_.x_dot = static_cast<double>(msg->x);
      xc_.y_dot = static_cast<double>(msg->y);
      // xc_.z_dot determined by control
      xc_.r = static_cast<double>(msg->z);
      break;
    case rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ:
      control_mode_ = msg->mode;
      xc_.pn = 0.0;
      xc_.pe = 0.0;
      xc_.pd = 0.0;
      // xc_.phi determined by control
      // xc_.theta determined by control
      xc_.psi = 0.0;
      xc_.x_dot = 0.0;
      xc_.y_dot = 0.0;
      xc_.z_dot = 0.0;
      xc_.r = static_cast<double>(msg->z);

      xc_.ax = static_cast<double>(msg->x);
      xc_.ay = static_cast<double>(msg->y);
      xc_.az = static_cast<double>(msg->F);
      break;
  case rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE: // effectively pass right through to
      // ROSflight via the innermost control "loop"
      control_mode_ = msg->mode;
      xc_.phi = static_cast<double>(msg->x);
      xc_.theta = static_cast<double>(msg->y);
      xc_.r = static_cast<double>(msg->z);
      xc_.throttle = static_cast<double>(msg->F);
      break;
    default:
      ROS_ERROR("roscopter/controller: Unhandled command message of type %d",
                msg->mode);
      break;
  }

  if (!received_cmd_)
    received_cmd_ = true;

}

void Controller::reconfigure_callback(roscopter::ControllerConfig& config,
                                      uint32_t level)
{
  double P, I, D, tau;
  tau = config.tau;
  P = config.x_dot_P;
  I = config.x_dot_I;
  D = config.x_dot_D;
  PID_x_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

  P = config.y_dot_P;
  I = config.y_dot_I;
  D = config.y_dot_D;
  PID_y_dot_.setGains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

  P = config.z_dot_P;
  I = config.z_dot_I;
  D = config.z_dot_D;
  // set max z accelerations so that we can't fall faster than 1 gravity
  PID_z_dot_.setGains(P, I, D, tau, 1.0, -max_accel_z_);

  P = config.north_P;
  I = config.north_I;
  D = config.north_D;
  max_.n_dot = config.max_n_dot;
  PID_n_.setGains(P, I, D, tau, max_.n_dot, -max_.n_dot);

  P = config.east_P;
  I = config.east_I;
  D = config.east_D;
  max_.e_dot = config.max_e_dot;
  PID_e_.setGains(P, I, D, tau, max_.e_dot, -max_.e_dot);

  P = config.down_P;
  I = config.down_I;
  D = config.down_D;
  max_.d_dot = config.max_d_dot;
  PID_d_.setGains(P, I, D, tau, max_.d_dot, -max_.d_dot);

  P = config.psi_P;
  I = config.psi_I;
  D = config.psi_D;
  PID_psi_.setGains(P, I, D, tau);

  max_.roll = config.max_roll;
  max_.pitch = config.max_pitch;
  max_.yaw_rate = config.max_yaw_rate;
  max_.throttle = config.max_throttle;

  max_.n_dot = config.max_n_dot;
  max_.e_dot = config.max_e_dot;
  max_.d_dot = config.max_d_dot;

  throttle_eq_ = config.equilibrium_throttle;

  ROS_INFO("new gains");

  resetIntegrators();
}


void Controller::computeControl(double dt)
{
//std::cout << "IN THE CONTROL LOOP" << std::endl;
  if(dt <= 0.0000001)
  {
    // This messes up the derivative calculation in the PID controllers
    ROS_WARN("dt <= 0.0000001; no control computed");
    return;
  }

  uint8_t mode_flag = control_mode_;

  if (mode_flag == rosflight_msgs::Command::MODE_PITCH_YVEL_YAW_ALTITUDE)
  {
      // Compute desired accelerations (in terms of g's) in the vehicle 1 frame
      // Rotate body frame velocities to vehicle 1 frame velocities
      double sinp = sin(xhat_.phi);
      double cosp = cos(xhat_.phi);
      double sint = sin(xhat_.theta);
      double cost = cos(xhat_.theta);
      double pydot = cosp * xhat_.v - sinp * xhat_.w;
      double pddot =
          -sint * xhat_.u + sinp * cost * xhat_.v + cosp * cost * xhat_.w;

      /// Calculate desired yaw rate
      // First, determine the shortest direction to the commanded psi
      if(fabs(xc_.psi + 2*M_PI - xhat_.psi) < fabs(xc_.psi - xhat_.psi))
      {
        xc_.psi += 2*M_PI;
      }
      else if (fabs(xc_.psi - 2*M_PI -xhat_.psi) < fabs(xc_.psi - xhat_.psi))
      {
        xc_.psi -= 2*M_PI;
      }
      xc_.r = PID_psi_.computePID(xc_.psi, xhat_.psi, dt);

      /// Get commanded accelerations
      xc_.ax = 0.0;
      xc_.ay = PID_y_dot_.computePID(xc_.y_dot, pydot, dt);
      double pddot_c = PID_d_.computePID(xc_.pd, xhat_.pd, dt, pddot);
      xc_.z_dot = pddot_c;
      xc_.az = PID_z_dot_.computePID(pddot_c, pddot, dt);

      /// Get commanded phi and throttle from the total commanded acceleration
      // Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
      double total_acc_c = sqrt((1.0 - xc_.az) * (1.0 - xc_.az) +
                                xc_.ax * xc_.ax + xc_.ay * xc_.ay);  // (in g's)
      if (total_acc_c > 0.001)
        xc_.phi = asin(xc_.ay / total_acc_c);
      else
        xc_.phi = 0;

      // Compute desired thrust based on current pose
      xc_.throttle = (1.0 - xc_.az) * throttle_eq_ / cosp / cost;

      mode_flag = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_XPOS_YPOS_YAW_ALTITUDE)
  {
    // Figure out desired velocities (in inertial frame)
    // By running the position controllers
    double pndot_c = PID_n_.computePID(xc_.pn, xhat_.pn, dt);
    double pedot_c = PID_e_.computePID(xc_.pe, xhat_.pe, dt);

    // Calculate desired yaw rate
    // First, determine the shortest direction to the commanded psi
    if(fabs(xc_.psi + 2*M_PI - xhat_.psi) < fabs(xc_.psi - xhat_.psi))
    {
      xc_.psi += 2*M_PI;
    }
    else if (fabs(xc_.psi - 2*M_PI -xhat_.psi) < fabs(xc_.psi - xhat_.psi))
    {
      xc_.psi -= 2*M_PI;
    }
    xc_.r = PID_psi_.computePID(xc_.psi, xhat_.psi, dt);

    xc_.x_dot = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);
    xc_.y_dot = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi);

    mode_flag = rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE)
  {
    // Compute desired accelerations (in terms of g's) in the vehicle 1 frame
    // Rotate body frame velocities to vehicle 1 frame velocities
    double sinp = sin(xhat_.phi);
    double cosp = cos(xhat_.phi);
    double sint = sin(xhat_.theta);
    double cost = cos(xhat_.theta);
    double pxdot =
        cost * xhat_.u + sinp * sint * xhat_.v + cosp * sint * xhat_.w;
    double pydot = cosp * xhat_.v - sinp * xhat_.w;
    double pddot =
        -sint * xhat_.u + sinp * cost * xhat_.v + cosp * cost * xhat_.w;

    xc_.ax = PID_x_dot_.computePID(xc_.x_dot, pxdot, dt);
    xc_.ay = PID_y_dot_.computePID(xc_.y_dot, pydot, dt);

    // Nested Loop for Altitude
    double pddot_c = PID_d_.computePID(xc_.pd, xhat_.pd, dt, pddot);
    xc_.z_dot = pddot_c;
    xc_.az = PID_z_dot_.computePID(pddot_c, pddot, dt);
    mode_flag = rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ)
  {
    // Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
    double total_acc_c = sqrt((1.0 - xc_.az) * (1.0 - xc_.az) +
                              xc_.ax * xc_.ax + xc_.ay * xc_.ay);  // (in g's)
    if (total_acc_c > 0.001)
    {
      xc_.phi = asin(xc_.ay / total_acc_c);
      xc_.theta = -1.0*asin(xc_.ax / total_acc_c);
    }
    else
    {
      xc_.phi = 0;
      xc_.theta = 0;
    }

    // Compute desired thrust based on current pose
    double cosp = cos(xhat_.phi);
    double cost = cos(xhat_.theta);
    xc_.throttle = (1.0 - xc_.az) * throttle_eq_ / cosp / cost;

    mode_flag = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
  }

  if(mode_flag == rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
  {
    // Pack up and send the command
    command_.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    command_.F = saturate(xc_.throttle, max_.throttle, 0.0);
    command_.x = saturate(xc_.phi, max_.roll, -max_.roll);
    command_.y = saturate(xc_.theta, max_.pitch, -max_.pitch);
    command_.z = saturate(xc_.r, max_.yaw_rate, -max_.yaw_rate);
    if (armed_)
        command_.ignore = 0;
    else
        command_.ignore = 8;

    if (-xhat_.pd < min_altitude_)
    {
      command_.x = 0.;
      command_.y = 0.;
      command_.z = 0.;
    }
  }
}

void Controller::publishCommand()
{
//    std::cout << "ROSCOPTER publish COMMAND" << std::endl;
  command_.header.stamp = ros::Time::now();
  command_pub_.publish(command_);
  nav_msgs::Odometry full_cmd;
  full_cmd.header.stamp = ros::Time::now();
  full_cmd.pose.pose.position.x = xc_.pn;
  full_cmd.pose.pose.position.y = xc_.pe;
  full_cmd.pose.pose.position.z = xc_.pd;
  full_cmd.pose.pose.orientation.x = xc_.phi;
  full_cmd.pose.pose.orientation.y = xc_.theta;
  full_cmd.pose.pose.orientation.z = xc_.psi;
  // NO W USED BECAUSE EULER CONVENTION USED
  full_cmd.twist.twist.linear.x = xc_.x_dot;
  full_cmd.twist.twist.linear.y = xc_.y_dot;
  full_cmd.twist.twist.linear.z = xc_.z_dot;
  // NO P OR Q USED BECAUSE ONLY R USED
  full_cmd.twist.twist.angular.z = xc_.r;
  full_command_pub_.publish(full_cmd);
}

void Controller::resetIntegrators()
{
  PID_x_dot_.clearIntegrator();
  PID_y_dot_.clearIntegrator();
  PID_z_dot_.clearIntegrator();
  PID_n_.clearIntegrator();
  PID_e_.clearIntegrator();
  PID_d_.clearIntegrator();
  PID_psi_.clearIntegrator();
}

double Controller::saturate(double x, double max, double min)
{
  x = (x > max) ? max : x;
  x = (x < min) ? min : x;
  return x;
}

}  // namespace controller
