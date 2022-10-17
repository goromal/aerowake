#include <stdio.h>
#include <iostream>
#include <math.h>
#include <chrono>

#include <mav_msgs/common.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "unscented_kalman_filter/unscented_kalman_filter.h"

namespace unscented_kalman_filter {

UnscentedKalmanFilter::UnscentedKalmanFilter(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : force_offset_fsm_(nh_private),
      initialized_parameters_(false),
      filter_active_(false),
      nh_(nh),
      nh_private_(nh_private),
      GRAVITY(9.8065),
      DEBUGMODE(false)
{
  InitializeParameters();
}

UnscentedKalmanFilter::~UnscentedKalmanFilter()
{
}

void UnscentedKalmanFilter::InitializeParameters()
{
  ROS_INFO_STREAM("Unscented kalman filter: initializing parameters");

  // Loading parameters
  loadParameter<double>(nh_private_, "sampling_time", &uav_params_.dT);
  loadParameter<int>(nh_private_, "num_rotors", &num_rotors_);

  // Initialization of motorRpm
//  rotors_speed_measurement_ = Eigen::VectorXd::Zero(6); // <<<<<<<<<<<<<
//  rotors_speed_measurement_.resize(num_rotors_, 1);
//  rotors_speed_measurement_.setZero();
  rotors_speed_measurement_ = Eigen::MatrixXd::Zero(num_rotors_, 1);

  // Vehicle parameters
  loadParameter<double>(nh_private_, "mass", &uav_params_.mass);
  // loadParameter<double>(nh_private_, "vehicle_param/arm_length", &uav_params_.l);
  loadParameter<double>(nh_private_, "linear_mu", &uav_params_.k_drag);
  loadParameter<double>(nh_private_, "rotor_inertia", &uav_params_.I_r);
//  loadParameter<double>(nh_private_, "motor/kn", &uav_params_.k_n);
//  loadParameter<double>(nh_private_, "motor/km", &uav_params_.k_m);

  std::vector<double> inertia_list;
  double Ixx, Iyy, Izz;
  loadParameter<double>(nh_private_, "uav_ixx", &Ixx);
  loadParameter<double>(nh_private_, "uav_iyy", &Iyy);
  loadParameter<double>(nh_private_, "uav_izz", &Izz);
  inertia_list.push_back(Ixx);
  inertia_list.push_back(Iyy);
  inertia_list.push_back(Izz);
//  loadParameter<std::vector<double>>(nh_private_, "vehicle_param/inertia", &inertia_list);

  uav_params_.inertia.diagonal() = Eigen::Vector3d(inertia_list.data());
  ROS_INFO_STREAM("Vehicle inertia matrix (diagonal):\n"<<uav_params_.inertia.diagonal());

  // Adding Rotor inertia to Izz entry of inertia matrix
//  uav_params_.inertia.diagonal()[2] += rotors_speed_measurement_.size() * uav_params_.I_r; // <<<<
  uav_params_.inertia.diagonal()[2] += rotors_speed_measurement_.rows() * uav_params_.I_r;
  uav_params_.g = GRAVITY;

  // Process noise covariance matrix Q
  double Q_position, Q_velocity, Q_attitude, Q_angularvelocity,
    Q_external_FxFy, Q_external_Fz, Q_external_Myaw;
  loadParameter<double>(nh_private_, "Q_position", &Q_position);
  loadParameter<double>(nh_private_, "Q_velocity", &Q_velocity);
  loadParameter<double>(nh_private_, "Q_attitude", &Q_attitude);
  loadParameter<double>(nh_private_, "Q_angularvelocity", &Q_angularvelocity);
  loadParameter<double>(nh_private_, "Q_external_FxFy", &Q_external_FxFy);
  loadParameter<double>(nh_private_, "Q_external_Fz", &Q_external_Fz);
  loadParameter<double>(nh_private_, "Q_external_Myaw", &Q_external_Myaw);
  Q_.setZero();
  Q_.diagonal() << Eigen::Vector3d::Constant(Q_position),
        Eigen::Vector3d::Constant(Q_velocity),
        Eigen::Vector3d::Constant(Q_attitude),
        Eigen::Vector3d::Constant(Q_angularvelocity),
        Eigen::Vector2d::Constant(Q_external_FxFy), Q_external_Fz,
        Q_external_Myaw;
  ROS_INFO_STREAM("Process noise covariance matrix Q:\n"<<Q_);

  // Measurement noise covariance matrix R
  double R_position, R_velocity, R_attitude, R_angularvelocity;
  loadParameter<double>(nh_private_, "R_position", &R_position);
  loadParameter<double>(nh_private_, "R_velocity", &R_velocity);
  loadParameter<double>(nh_private_, "R_attitude", &R_attitude);
  loadParameter<double>(nh_private_, "R_angularvelocity", &R_angularvelocity);
  R_.setZero();
  R_.diagonal() << Eigen::Vector3d::Constant(R_position),
       Eigen::Vector3d::Constant(R_velocity),
       Eigen::Vector3d::Constant(R_attitude),
       Eigen::Vector3d::Constant(R_angularvelocity);
  ROS_INFO_STREAM("Measurement noise covariance matrix R:\n"<<R_);

  // computation of allocation matrix <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  std::vector<double> rotor_positions(static_cast<unsigned long>(3 * num_rotors_));
  std::vector<double> rotor_vector_normal(static_cast<unsigned long>(3 * num_rotors_));
  std::vector<int> rotor_rotation_directions(static_cast<unsigned long>(num_rotors_));
  std::vector<double> motor_kms(static_cast<unsigned long>(num_rotors_));
  std::vector<double> motor_kns(static_cast<unsigned long>(num_rotors_));

  ROS_ASSERT(nh_private_.getParam("rotor_positions", rotor_positions));
  ROS_ASSERT(nh_private_.getParam("rotor_vector_normal", rotor_vector_normal));
  ROS_ASSERT(nh_private_.getParam("rotor_rotation_directions", rotor_rotation_directions));
  ROS_ASSERT(nh_private_.getParam("km", motor_kms));
  ROS_ASSERT(nh_private_.getParam("kn", motor_kns));

//  double k_n = uav_params_.k_n, k_m = uav_params_.k_m;

  // Rotation matrix from "forward right down" body frame convention to "forward left up"
  Eigen::Matrix3d R;                                                    // <>
  R << 1.0,  0.0,  0.0,                                                 // <>
       0.0, -1.0,  0.0,                                                 // <>
       0.0,  0.0, -1.0;                                                 // <>

  allocation_matrix_ = Eigen::MatrixXd::Zero(4, num_rotors_);

  for (int i = 0; i < num_rotors_; i++)
  {
      double k_n = motor_kns[static_cast<unsigned long>(i)];
      double k_m = motor_kms[static_cast<unsigned long>(i)];
      Eigen::Vector3d rotor_position;
      Eigen::Vector3d rotor_normal;
      int rotor_direction = rotor_rotation_directions[static_cast<unsigned long>(i)];

      for (int j = 0; j < 3; j++)
      {
          rotor_position(j) = rotor_positions[static_cast<unsigned long>(3*i + j)];
          rotor_normal(j) = rotor_vector_normal[static_cast<unsigned long>(3*i + j)];
      }
      rotor_normal.normalize();

      // frame transformation calculations
      rotor_direction *= -1;                                            // <>
      rotor_position = R * rotor_position;                              // <>
      rotor_normal = R * rotor_normal;                                  // <>

      // matrix computation
      Eigen::Vector3d moment_from_thrust = rotor_position.cross(rotor_normal);
      Eigen::Vector3d moment_from_torque = rotor_direction * rotor_normal;

      allocation_matrix_(0, i) = k_n * moment_from_thrust(0) + k_m * moment_from_torque(0); // l
      allocation_matrix_(1, i) = k_n * moment_from_thrust(1) + k_m * moment_from_torque(1); // m
      allocation_matrix_(2, i) = k_n * moment_from_thrust(2) + k_m * moment_from_torque(2); // n
      allocation_matrix_(3, i) = k_n * rotor_normal(2);                                     // F
  }

//  const double c = 0.866025403784;  //cos(30)
//  const double s = 0.5;       //sin(30)

//  allocation_matrix_ << s,   1.0, s,    -s,  -1.0, -s,
//                       -c,   0.0, c,     c,  0.0,  -c,
//                       -1.0, 1.0, -1.0, 1.0, -1.0, 1.0,
//                       1.0,  1.0, 1.0,  1.0, 1.0,  1.0;

//  double /*l = uav_params_.l,*/ k_n = uav_params_.k_n, k_m = uav_params_.k_m;
//  double paramArray[4] = { /*l **/ k_n, /*l **/ k_n, k_n * k_m, k_n };
//  for (int i = 0; i < 4; i++) {
//    allocation_matrix_.row(i) = paramArray[i] * allocation_matrix_.row(i);
//  }
  ROS_INFO_STREAM("Allocation matrix:\n"<<allocation_matrix_);
  // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

  //Initialization of odometry_measurement_
  odometry_measurement_ = mav_msgs::EigenOdometry();

  // computation of input state estimate as a vector with linar dynamic in W frame
  // and rotational dynamic in B frame
  z_curr_ = Eigen::VectorXd::Zero(kMeasurementSize);

  // Initializaiton of state vector X
  x_prev_ = Eigen::VectorXd::Zero(kStateSize);

  // Initialize P matrices
  P_0_ = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  P_0_.diagonal() << 1, 1, 1, 0.01, 0.01, 0.01, 0.001, 0.001, 0.001, 0.01, 0.01, 0.01, 1.5, 1.5, 1.5, 0.01;
  P_prev_ = P_0_;
  ROS_INFO_STREAM("Initial state covariance matrix P(0):\n"<<P_0_);

  // initialize H
  H_ = Eigen::MatrixXd::Zero(kMeasurementSize, kStateSize);
  H_.block(0, 0, kMeasurementSize, kMeasurementSize) = Eigen::MatrixXd::Identity(kMeasurementSize,
                                                                                 kMeasurementSize);
  ROS_INFO_STREAM("Update Step Linear Model Matrix: \n"<<H_);

  // initialization of the Kalman Filter gain, preallocated for speed
  K_ = Eigen::MatrixXd::Zero(kStateSize, kMeasurementSize);

  // initialize vector for propagation of sigma points
  temp_out_ = Eigen::VectorXd::Zero(kSigmaPointsSize);

  // initialization of attitude quaternion for USQUE
  attitude_quat_prev_.setIdentity();

  advanced_quat_averaging_ = false;
  impose_quaternion_positivity_ = true;

  //initialize parameters for the unscented transformation
  initializeUnscentedTransformation();

  ROS_INFO_STREAM("Unscented kalman filter: initialization completed");

  force_offset_fsm_.start();
}

template<typename T> void UnscentedKalmanFilter::loadParameter(const ros::NodeHandle& nh,
                                                               const std::string parameter_name,
                                                               T* paramater_destination)
{
  if (!nh.getParam(parameter_name, *paramater_destination)) {
    ROS_ERROR("Unscented kalman filter: unable to load parameter \'%s\'", parameter_name.c_str());
    abort();
  }
}

void UnscentedKalmanFilter::SetOdometryMeasurement(const mav_msgs::EigenOdometry& odometry)
{
  ROS_INFO_STREAM_ONCE("Unscented kalman filter: received a first odometry measurement.\n");
  odometry_measurement_ = odometry;
}

void UnscentedKalmanFilter::setProcessNoiseCovariance(const Eigen::MatrixXd& Q)
{
  ROS_INFO_STREAM("Unscented kalman filter: update process noise covariance matrix.\n"<<Q);
  Q_ = Q;
}

void UnscentedKalmanFilter::setMeasurementNoiseCovariance(const Eigen::MatrixXd& R)
{
  ROS_INFO_STREAM("Unscented kalman filter: update measurement noise covariance matrix\n\n"<<R);
  R_ = R;
}

void UnscentedKalmanFilter::setAlphaBetaKappaMrpA(const double alpha, const double beta,
                                                  const double kappa, const double MRP_a)
{
  alpha_ = alpha;
  beta_ = beta;
  kappa_ = kappa;
  ROS_INFO_STREAM(
      "Updated Parameters for the Unscented Transformation:  \n alpha: "<<alpha_ <<"\nbeta: "<<beta_<<"\nkappa: "<<kappa_);

  param_a_ = MRP_a;
  param_f_ = 2 * (param_a_ + 1);
  ROS_INFO_STREAM(
      "Updated Modified Rodriguez Parameter: \nMRP_a: " << param_a_ << "\n MRP_f: "<< param_f_);

  return;
}

void UnscentedKalmanFilter::setSettings(const bool second_order_dynamic, const bool debug_mode,
                                        const bool advance_quat_averaging)
{
  ROS_INFO_STREAM(
      "Updated UKF settings: \n Use second Order dynamics:"<< second_order_dynamic << "\nDebug mode: "<< debug_mode << "\n advanced quaternon averaging: "<< advance_quat_averaging);

  second_order_dynamics_ = second_order_dynamic;
  DEBUGMODE = debug_mode;
  advanced_quat_averaging_ = advance_quat_averaging;
  return;
}

geometry_msgs::WrenchStamped UnscentedKalmanFilter::getUkfExternalForcesMoments()
{
  Eigen::Vector3d force_estimate, current_offset;
  current_offset << 0.0, 0.0, 0.0;
  force_estimate << x_curr_(kExternalForcesIndex), x_curr_(kExternalForcesIndex + 1), x_curr_(kExternalForcesIndex + 2);
  force_offset_fsm_.getCurrentOffset(force_estimate, &current_offset);

  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.wrench.force.x = x_curr_(kExternalForcesIndex) - current_offset(0);
  msg.wrench.force.y = x_curr_(kExternalForcesIndex + 1) - current_offset(1);;
  msg.wrench.force.z = x_curr_(kExternalForcesIndex + 2) - current_offset(2);;
  msg.wrench.torque.x = 0.0;
  msg.wrench.torque.y = 0.0;
  msg.wrench.torque.z = x_curr_(kExternalYawMomentsIndex);
  return msg;
}

void UnscentedKalmanFilter::getUkfEstimatedOdometry(nav_msgs::Odometry* ukf_estimated_odometry)
{
  ukf_estimated_odometry->header.stamp = ros::Time::now();
  ukf_estimated_odometry->header.frame_id = "world";
  ukf_estimated_odometry->pose.pose.position.x = x_curr_(kPositionIndex);
  ukf_estimated_odometry->pose.pose.position.y = x_curr_(kPositionIndex + 1);
  ukf_estimated_odometry->pose.pose.position.z = x_curr_(kPositionIndex + 2);

  ukf_estimated_odometry->pose.pose.orientation.w = attitude_quat_curr_.w();
  ukf_estimated_odometry->pose.pose.orientation.x = attitude_quat_curr_.x();
  ukf_estimated_odometry->pose.pose.orientation.y = attitude_quat_curr_.y();
  ukf_estimated_odometry->pose.pose.orientation.z = attitude_quat_curr_.z();

  ukf_estimated_odometry->twist.twist.linear.x = x_curr_(kVelocityIndex);
  ukf_estimated_odometry->twist.twist.linear.y = x_curr_(kVelocityIndex + 1);
  ukf_estimated_odometry->twist.twist.linear.z = x_curr_(kVelocityIndex + 2);

  ukf_estimated_odometry->twist.twist.angular.x = x_curr_(kAngularRateIndex);
  ukf_estimated_odometry->twist.twist.angular.y = x_curr_(kAngularRateIndex + 1);
  ukf_estimated_odometry->twist.twist.angular.z = x_curr_(kAngularRateIndex + 2);

  return;
}

geometry_msgs::WrenchStamped UnscentedKalmanFilter::getForcesTorqueCovariance()
{
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.wrench.force.x = P_curr_(kExternalForcesIndex, kExternalForcesIndex);
  msg.wrench.force.y = P_curr_(kExternalForcesIndex + 1, kExternalForcesIndex + 1);
  msg.wrench.force.z = P_curr_(kExternalForcesIndex + 2, kExternalForcesIndex + 2);
  msg.wrench.torque.x = 0;
  msg.wrench.torque.y = 0;
  msg.wrench.torque.z = P_curr_(kExternalYawMomentsIndex, kExternalYawMomentsIndex);
  return msg;
}

void UnscentedKalmanFilter::simulateSystemDynamics(const Eigen::VectorXd& x, Eigen::VectorXd* out)
{

  const double dT = uav_params_.dT;

  // Computation of the attitude quaternion
  Eigen::Quaterniond attitude_quaternion;
  attitude_quaternion.vec() = x.segment(kAttitudeXYZIndex, 3);
  attitude_quaternion.w() = x(kAttitudeWIndex);

  // Computation of rotational dynamics
  double thrust = 0;
  Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();
  RotationalDynamicsAndThrust(x.segment(kAngularRateIndex, 3), x(kExternalYawMomentsIndex), &thrust,
                              &angular_acceleration);

  // Computation of translational dynamics
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  TranslationalDynamics(attitude_quaternion, x.segment(kVelocityIndex, 3),
                        x.segment(kExternalForcesIndex, 3), thrust, &linear_acceleration);

  // - Dynamic of the external forces and moments
  Eigen::Vector3d Fextdot(0, 0, 0);
  double Mextdot = 0;

  // Integration using Forward Euler integration
  // position
  out->segment(kPositionIndex, 3) = x.segment(kPositionIndex, 3)
      + x.segment(kVelocityIndex, 3) * dT;

  // Velocity
  out->segment(kVelocityIndex, 3) = x.segment(kVelocityIndex, 3) + linear_acceleration * dT;

  // Attitude
  Eigen::Quaterniond integrated_attitude_quaternion;
  multiStepFullQuaternionIntegration(dT, kIntegrationStepQuaternionKinematic, attitude_quaternion,
                                     x.segment(kAngularRateIndex, 3),
                                     &integrated_attitude_quaternion);
  out->segment(kAttitudeXYZIndex, 3) = integrated_attitude_quaternion.vec();
  (*out)(kAttitudeWIndex) = integrated_attitude_quaternion.w();

  //angular velocity
  out->segment(kAngularRateIndex, 3) = x.segment(kAngularRateIndex, 3) + dT * angular_acceleration;

  //Integrate external forces and moments
  out->segment(kExternalForcesIndex, 3) = x.segment(kExternalForcesIndex, 3) + dT * Fextdot;
  (*out)(kExternalYawMomentsIndex) = x(kExternalYawMomentsIndex) + dT * Mextdot;
}

void UnscentedKalmanFilter::integrateFullQuaternionKinematics3(
    const Eigen::Vector4d& attitude_quaternion, const Eigen::Vector3d& omega,
    Eigen::Vector4d *integrated_attitude_quaternion, double dT)
{
  if (omega.norm() == 0) {
    *integrated_attitude_quaternion = attitude_quaternion;
    return;
  }
  Eigen::Matrix4d Omega = Eigen::Matrix4d::Identity() * cos(0.5 * omega.norm() * dT);
  Omega.block<1, 3>(3, 0) = -(sin(0.5 * omega.norm() * dT) * omega.normalized()).transpose();
  Omega.block<3, 1>(0, 3) = sin(0.5 * omega.norm() * dT) * omega.normalized();

  *integrated_attitude_quaternion = (Omega * attitude_quaternion).normalized();
}

void UnscentedKalmanFilter::multiStepFullQuaternionIntegration(
    double end_time, int iterations, const Eigen::Quaterniond attitude_quaternion,
    const Eigen::Vector3d& omega, Eigen::Quaterniond *integrated_attitude_quaternion)
{
  Eigen::Vector4d temp_attitude_quaternion, temp_integrated_attitude_quaternion;

  temp_attitude_quaternion.segment(0, 3) = attitude_quaternion.vec();
  temp_attitude_quaternion(3) = attitude_quaternion.w();
  double dT = end_time / iterations;
  for (int i = 0; i < iterations; i++) {
    integrateFullQuaternionKinematics3(temp_attitude_quaternion, omega,
                                       &temp_integrated_attitude_quaternion, dT);
    temp_attitude_quaternion = temp_integrated_attitude_quaternion;
  }

  integrated_attitude_quaternion->vec() = temp_integrated_attitude_quaternion.segment(0, 3);
  integrated_attitude_quaternion->w() = temp_integrated_attitude_quaternion(3);
  return;
}

void UnscentedKalmanFilter::RotationalDynamicsAndThrust(const Eigen::Vector3d& angular_velocity_B,
                                                        const double external_moment_B,
                                                        double* thrust,
                                                        Eigen::Vector3d* angular_acceleration_B)
{

//  const int n_rotor = 6;
  const Eigen::VectorXd propeller_torque_B = allocation_matrix_
      * (rotors_speed_measurement_.cwiseAbs().cwiseProduct(rotors_speed_measurement_.cwiseAbs()));

  // thrust scalar
  *thrust = propeller_torque_B(3);

  // change in angular accleration due to control input
  double propeller_residual_angular_speed = rotors_speed_measurement_.sum();
  Eigen::Vector3d change_orientation_propeller_plane_B;
  change_orientation_propeller_plane_B(0) = uav_params_.I_r * angular_velocity_B(1)
      * (propeller_residual_angular_speed - num_rotors_ * angular_velocity_B(2));
  change_orientation_propeller_plane_B(1) = -uav_params_.I_r * angular_velocity_B(0)
      * (propeller_residual_angular_speed - num_rotors_ * angular_velocity_B(2));
  change_orientation_propeller_plane_B(2) = 0;

  // creation of torque vecotr
  Eigen::Vector3d external_moments_B;
  external_moments_B(0) = 0;
  external_moments_B(1) = 0;
  external_moments_B(2) = external_moment_B;
  // rotational dynamics equation (inertia normalized)
  if (second_order_dynamics_ == true) {

    *angular_acceleration_B = uav_params_.inertia.inverse()
        * (propeller_torque_B.segment<3>(0)
            - angular_velocity_B.cross(uav_params_.inertia * angular_velocity_B)
            - change_orientation_propeller_plane_B + external_moments_B);
  } else {

    *angular_acceleration_B = uav_params_.inertia.inverse()
        * (propeller_torque_B.segment<3>(0) + external_moments_B);
  }
  return;
}
;

void UnscentedKalmanFilter::TranslationalDynamics(const Eigen::Quaterniond& attitude_quaternion,
                                                  const Eigen::Vector3d& linear_velocity_I,
                                                  const Eigen::Vector3d& external_forces_I,
                                                  const double thrust,
                                                  Eigen::Vector3d* linear_acceleration_I)
{
  Eigen::Matrix3d R_IB = attitude_quaternion.toRotationMatrix();

  Eigen::Vector3d linear_velocity_B, drag_force_B, thrust_force_B, gravity_acceleration_I;

  // drag force
  Eigen::Matrix3d selection_matrix_xy;
  selection_matrix_xy << 1, 0, 0, 0, 1, 0, 0, 0, 0;

  linear_velocity_B = R_IB.transpose() * linear_velocity_I;

  drag_force_B = -uav_params_.k_drag * rotors_speed_measurement_.cwiseAbs().sum()
      * selection_matrix_xy * linear_velocity_B;  //.cwiseProduct(linear_velocity_B.cwiseAbs());

  // gravity force
  gravity_acceleration_I << 0, 0, -uav_params_.g;

  // thrust acceleration
  thrust_force_B << 0, 0, thrust;
  // translational dynamics, mass normalized
  if (second_order_dynamics_ == true)
    *linear_acceleration_I = 1 / uav_params_.mass * R_IB * (drag_force_B + thrust_force_B)
        + gravity_acceleration_I + 1 / uav_params_.mass * (external_forces_I);
  else
    *linear_acceleration_I = 1 / uav_params_.mass * R_IB * (thrust_force_B) + gravity_acceleration_I
        + 1 / uav_params_.mass * (external_forces_I);

  return;
}

void UnscentedKalmanFilter::SetMotorSpeedMeasurement(const mav_msgs::Actuators& actuators_msg)
{
  mav_msgs::EigenActuators actuators(Eigen::VectorXd::Zero(num_rotors_));
  mav_msgs::eigenActuatorsFromMsg(actuators_msg, &actuators);
  rotors_speed_measurement_ = actuators.angular_velocities.head(num_rotors_);
}

void UnscentedKalmanFilter::initializeUnscentedTransformation()
{
  // Load parameters for unscented transformation
  loadParameter<double>(nh_private_, "alpha", &alpha_);
  loadParameter<double>(nh_private_, "beta", &beta_);
  loadParameter<double>(nh_private_, "kappa", &kappa_);
  loadParameter<double>(nh_private_, "MRP_a", &param_a_);
  param_f_ = 2 * (param_a_ + 1);
  ROS_INFO_STREAM("USQUE: \nparam a:"<< param_a_ <<"\nparam f:"<<param_f_);

  // building matrices of weights
  // parameters for the unscented transformation
  int input_size = kStateSize;
  double lambda = (alpha_ * alpha_) * (input_size + kappa_) - input_size;
  gamma_ = lambda + input_size;

  ROS_INFO_STREAM("UT: lambda: " << lambda);
  ROS_INFO_STREAM("UT: gamma: " << gamma_);

  /* computation of sigma points */
  // weights for the mean and covariance
  diagWc_(0, 0) = lambda / gamma_;
  diagWm_(0, 0) = diagWc_(0, 0) + (1 - alpha_ * alpha_ + beta_);

  for (int i = 1; i <= 2 * input_size; i++) {
    diagWc_(i, i) = 1 / (2 * gamma_);
    diagWm_(i, i) = 1 / (2 * gamma_);
  }
  ROS_INFO_STREAM("Unscented transformation: weights for the mean:\n"<<diagWm_.diagonal().transpose());
  ROS_INFO_STREAM("Unscented transformation: weights for the covariance:\n"<<diagWc_.diagonal().transpose());
}

void UnscentedKalmanFilter::UpdateStatusEstimate()
{

  /* - Prediction step  -*/
  UsquePredictionStep();

  /* - Update Step -*/
  UsqueUpdateStep();

  /* - NaN sanity check -*/
  if (hasNaN(attitude_quat_curr_) || hasNaNv(x_curr_)) {

    ROS_WARN_STREAM(
        "Filter has tried to output NaNs. Resetting the state and covariance of the filter");
    ROS_WARN_STREAM(
        "Failed data: \n current state: \n "<< x_curr_ << "\n current covariance: \n"<< P_curr_ << "\n current attitude quat: \n  vec: \n"<< attitude_quat_curr_.vec() << "\n scal: \n " << attitude_quat_curr_.w());

    P_curr_ = P_0_;
    x_curr_ = Eigen::VectorXd::Zero(kStateSize);
    attitude_quat_curr_.setIdentity();

  }

  /* - updateing state variables at every succesfull iteration -*/
  P_prev_ = P_curr_;
  x_prev_ = x_curr_;
  attitude_quat_prev_ = attitude_quat_curr_;

  return;

}

void UnscentedKalmanFilter::UsquePredictionStep()
{

  auto begin = std::chrono::high_resolution_clock::now();
  ROS_INFO_STREAM_ONCE("using USQUE prediction step");
  if (DEBUGMODE)
    ROS_INFO_STREAM(
        "previous attitude vec()"<< attitude_quat_prev_.vec() << "\nw:" <<attitude_quat_prev_.w());

  /* - checking if covariance matrix is positive definite - */
  Eigen::LLT<Eigen::MatrixXd> lltOfS(gamma_ * (P_prev_));
  if (lltOfS.info() == 0) {
    //everything seems good
  } else {
    ROS_WARN_STREAM("Cholesky decomposition failed, resetting the state covariance matrix to P_0_");
    ROS_WARN_STREAM("failed matrix:\n "<<gamma_*P_prev_);
    Eigen::LLT<Eigen::MatrixXd> lltOfS(gamma_ * P_prev_);
  }
  L_ = lltOfS.matrixL();  // retrieve factor L  in the decomposition
  if (DEBUGMODE)
    ROS_INFO_STREAM("cholesky decomposition" << L_);

  /* - computing the sigma points -*/
  // the function will set the value in sigma_points
  compute_sigma_points(x_prev_, attitude_quat_prev_, L_);  // set value in:  sigma_points_
  if (DEBUGMODE)
    ROS_INFO_STREAM("sigma point successfully computed\n"<< sigma_points_);

  // 0.3ms

  /* - propagating the sigma points -*/
  for (int i = 0; i < kSigmaPointsNum; i++) {
    simulateSystemDynamics(sigma_points_.col(i), &temp_out_);
    sigma_points_.col(i) = temp_out_;
  }
  if (DEBUGMODE)
    ROS_INFO_STREAM("sigma points sucessfully propagated in system dynamics\n"<< sigma_points_);

  auto end = std::chrono::high_resolution_clock::now();

  /* Calculate the a priori estimate mean, deltas and covariance. */
  compute_sigma_point_mean(sigma_points_);  // compute value for usque_x_curr_pred_
  if (DEBUGMODE)
    ROS_INFO_STREAM_ONCE("sigma points mean successfully computed\n" << usque_x_curr_pred_);

  compute_sigma_point_deltas(sigma_points_, usque_x_curr_pred_);  // compute delta vector between each sigma point and the mean; set value in w_prime_
  if (DEBUGMODE)
    ROS_INFO_STREAM("sigma points deltas\n " << w_prime_);

  compute_sigma_point_deltas_mean(w_prime_);
  if (DEBUGMODE)
    ROS_INFO_STREAM("predicted state vector (with delta quaternion)\n"<< x_curr_pred_);

  /* - calculating the covariance based on delta sigma points - formula (8) Markley paper - */
  P_curr_pred_ = (w_prime_.colwise() - x_curr_pred_) * diagWc_
      * ((w_prime_.colwise() - x_curr_pred_).transpose()) + Q_;
  if (DEBUGMODE)
    ROS_INFO_STREAM("predicted covariance matrix\n"<< P_curr_pred_);

  if (advanced_quat_averaging_ == true) {

    attitude_quat_curr_pred_ = computeQuatAvg(sigma_points_, diagWm_);

  } else {

    attitude_quat_curr_pred_.vec() = sigma_points_.col(0).segment(kAttitudeXYZIndex, 3);
    attitude_quat_curr_pred_.w() = (sigma_points_.col(0))(kAttitudeWIndex);
  }
  if (DEBUGMODE)
    ROS_INFO_STREAM(
        "attitude quaternion \nvec:\n" << attitude_quat_curr_pred_.vec() << "\n scal:\n"<<attitude_quat_curr_pred_.w());
  if (DEBUGMODE)
    ROS_INFO_STREAM("prediction step computed");
  if (DEBUGMODE)
    std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() << "ns"
              << std::endl;

  return;

}

Eigen::Quaterniond UnscentedKalmanFilter::computeQuatAvg(const Eigen::MatrixXd& sigma_points,
                                                         const Eigen::MatrixXd& weights)
{
  /* Quaternion averaging, method based on "Quaternion Averaging" paper, from Markley, Cheng, Crassidis */

  Eigen::Quaterniond quat;
  quat.setIdentity();
  Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
  Eigen::Matrix<double, 4, 1> temp_vect_quat;

  // computation of M matrix
  for (int i = 0; i < kSigmaPointsNum; i++) {

    temp_vect_quat.segment(0, 3) = sigma_points.col(i).segment(kAttitudeXYZIndex, 3);
    temp_vect_quat(3) = sigma_points.col(i)(kAttitudeWIndex);
    M = M + weights(i, i) * (temp_vect_quat * (temp_vect_quat.transpose()));

  }

  if (DEBUGMODE)
    ROS_INFO_STREAM("M matrix for average quaternion\n"<< M);

  // looking for the dominant eigenvector
  Eigen::EigenSolver<Eigen::Matrix4d> es(M);
  double max_val = es.eigenvalues()[0].real();
  double max_index = 0;
  for (int i = 1; i < 4; i++) {

    if (es.eigenvalues()[i].real() > max_val) {
      max_index = i;
      max_val = es.eigenvalues()[i].real();
    } else if (es.eigenvalues()[i].real() == max_val) {
      ROS_WARN_STREAM("matrix has multiple equal eigenvalues!");
    }
  }

  if (DEBUGMODE) {
    ROS_INFO_STREAM("The eigenvalues of M are:" << "\n" << es.eigenvalues());
    ROS_INFO_STREAM("The matrix of eigenvectors, V, is:" << "\n" << es.eigenvectors() << "\n");
    ROS_INFO_STREAM("The selected eigenvector is:" << es.eigenvectors().col(max_index));
  }

  Eigen::Vector4d avg_q_vect = es.eigenvectors().col(max_index).real();

  // normalizing the dominant eigenvector
  if (avg_q_vect.norm() != 0)
    avg_q_vect.normalize();
  else {
    ROS_WARN_STREAM(
        "UKF: ERROR: The domiant eigenvector in advanced quaternion averaging is the null vector!\n resetting to identity");
    return quat;
  }

  // obtaining the quaternion from the vector
  quat.vec() << avg_q_vect.segment(0, 3);
  quat.w() = avg_q_vect(3);
  return quat;

}

void UnscentedKalmanFilter::compute_sigma_points(const Eigen::VectorXd& state,
                                                 const Eigen::Quaterniond & quat_state,
                                                 const Eigen::MatrixXd &L)
{

  // computing sigma points for all the states X = x+-L and considering quaternion as normal state
  sigma_points_.col(0).segment(0, kStateSize) = state;
  sigma_points_.block(0, 1, kStateSize, kStateSize) = L.colwise() + state;
  sigma_points_.block(0, kStateSize + 1, kStateSize, kStateSize) = -(L.colwise() - state);

  /*
   Construct error quaternions using the MRP method, equation 34 from the
   Markley paper.
   */
  //filling quaternion 0 column
  sigma_points_.col(0).segment(kAttitudeXYZIndex, 3) = quat_state.vec();
  sigma_points_.col(0)(kAttitudeWIndex) = quat_state.w();

  // compute a 4*kSigmaPointsNum matrix containng the sigma points quaternions.
  Eigen::Quaterniond temp_q;
  for (int i = 0; i < kStateSize; i++) {

    temp_q = computeQuaternionFromVectorUSQUE(L.col(i).segment(kAttitudeXYZIndex, 3)) * quat_state;
    sigma_points_.col(i + 1).segment(kAttitudeXYZIndex, 3) = temp_q.vec();
    (sigma_points_.col(i + 1))(kAttitudeWIndex) = temp_q.w();
  }

  for (int i = 0; i < kStateSize; i++) {

    temp_q = computeQuaternionFromVectorUSQUE(L.col(i).segment(kAttitudeXYZIndex, 3)).conjugate()
        * quat_state;
    sigma_points_.col(i + kStateSize + 1).segment(kAttitudeXYZIndex, 3) << temp_q.vec();
    (sigma_points_.col(i + kStateSize + 1))(kAttitudeWIndex) = temp_q.w();
  }
  return;
}

void UnscentedKalmanFilter::compute_sigma_point_mean(const Eigen::MatrixXd & sigma_points)
{

  Eigen::Quaterniond quat_sigma_points_mean;

  // mean for all the states
  usque_x_curr_pred_ = sigma_points * diagWm_ * Eigen::MatrixXd::Constant(kSigmaPointsNum, 1, 1);

  // mean for the quaternion sigma points -> simply compute the average of quaternion sigma points and normalizes the obtained quaternion
  quat_sigma_points_mean = compute_quaternion_sigma_point_mean(sigma_points);

  // storinq quaternions sigma points in the main vector
  usque_x_curr_pred_.segment(kAttitudeXYZIndex, 3) << quat_sigma_points_mean.vec();
  usque_x_curr_pred_(kAttitudeWIndex) = quat_sigma_points_mean.w();
  return;
}

Eigen::Quaterniond UnscentedKalmanFilter::compute_quaternion_sigma_point_mean(
    const Eigen::MatrixXd& sigma_points)
{
  Eigen::Matrix<double, kSigmaPointsSize, 1> sigma_points_mean;
  sigma_points_mean = sigma_points * diagWm_ * Eigen::MatrixXd::Constant(kSigmaPointsNum, 1, 1);
  Eigen::Quaterniond temp_q;
  temp_q.vec() << sigma_points_mean.segment(kAttitudeXYZIndex, 3);
  temp_q.w() = sigma_points_mean(kAttitudeWIndex);
  if (temp_q.norm() > 0)
    temp_q.normalize();
  else {
    ROS_WARN_STREAM("quaternion norm is zero! setting identity instead");
    temp_q.setIdentity();
  }

  return temp_q;
}

void UnscentedKalmanFilter::compute_sigma_point_deltas(const Eigen::MatrixXd & sigma_points,
                                                       const Eigen::VectorXd & mean)
{
  // compute sigma point delta for all the other states
  // formula
  if(DEBUGMODE){
    ROS_INFO_STREAM(
      "Sigma points without quaternion scalar part:\n" <<
      sigma_points.block(0, 0, kStateSize, kSigmaPointsNum));
  }
  w_prime_ = sigma_points.block(0, 0, kStateSize, kSigmaPointsNum);  // formula 38 + we are eliminating the last row

  if(DEBUGMODE)
    ROS_INFO_STREAM("Sigma points (w_prime):\n" << w_prime_);

  //which contains quaternion scalar part

  // compute sigma point deltas for quaternions

  // We will use the RPM to switch from a 4 state representation to the 3 states representation;
  // This explains why w_prime_ has a smaller number of rows (one less) than sigma_points.

  Eigen::Quaterniond quat_mean;
  quat_mean.w() = mean(kAttitudeWIndex);
  quat_mean.vec() = mean.segment(kAttitudeXYZIndex, 3);

  // equation 37b from Markley paper
  Eigen::Quaterniond sigmapoint_q;
  Eigen::Quaterniond delta_q;

  for (int i = 1; i < kSigmaPointsNum; i++) {

    sigmapoint_q.vec() = sigma_points.col(i).segment(kAttitudeXYZIndex, 3);
    sigmapoint_q.w() = (sigma_points.col(i))(kAttitudeWIndex);

    // eq 36
    delta_q = sigmapoint_q * quat_mean.conjugate();

    if (delta_q.w() == -param_a_) {
      ROS_WARN_STREAM("singular delta sigma point! changing its representaiton to the inverse");
      delta_q = delta_q.inverse();
    }
    w_prime_.col(i).segment(kAttitudeXYZIndex, 3) = quat2vecUsque(delta_q);
  }
  w_prime_.col(0).segment(kAttitudeXYZIndex, 3) << 0, 0, 0;  // (formula 37 a)

  if(DEBUGMODE){
    ROS_INFO_STREAM("Delta sigma points successfully computed.\n" << w_prime_);
  }
  return;
}

void UnscentedKalmanFilter::compute_sigma_point_deltas_mean(
    const Eigen::MatrixXd& sigma_points_deltas)
{
  // computing the mean of the delta sigma points (where attitude is represented using 3 states)
  // implementing formula (7) of Markley paper
  // storing the mean in x_curr_pred_

  x_curr_pred_ << sigma_points_deltas * diagWm_ * Eigen::VectorXd::Constant(kSigmaPointsNum, 1, 1);

  return;
}

void UnscentedKalmanFilter::UsqueUpdateStep()
{

  /* - mean of the measurement sigma points -*/
  Eigen::Vector3d measurement_mean_position = x_curr_pred_.segment(kPositionIndex, 3);
  Eigen::Vector3d measurement_mean_velocity = x_curr_pred_.segment(kVelocityIndex, 3);
  Eigen::Vector3d measurement_mean_angularrate = x_curr_pred_.segment(kAngularRateIndex, 3);

  /* - innovation vectors -*/
  Eigen::Vector3d innovation_position = odometry_measurement_.position_W
      - measurement_mean_position;
  Eigen::Vector3d innovation_velocity = odometry_measurement_.getVelocityWorld()
      - measurement_mean_velocity;

  // compute difference between previous attitude measurement and current attitude measurement (ideal correction that should be applied)
  // and transform it into vector using mrp (Rodriguez parameters)
  Eigen::Vector3d delta_attitude_measurement(
      quat2vecUsque((attitude_quat_prev_.conjugate()) * odometry_measurement_.orientation_W_B));
  Eigen::Vector3d innovation_attitude(
      delta_attitude_measurement - x_curr_pred_.segment(kAttitudeXYZIndex, 3));
  Eigen::Vector3d innovation_angularrate = odometry_measurement_.angular_velocity_B
      - measurement_mean_angularrate;

  // building innovation vector
  Eigen::VectorXd innovation = Eigen::Matrix<double, kMeasurementSize, 1>::Zero();
  innovation << innovation_position, innovation_velocity, innovation_attitude, innovation_angularrate;
  if (DEBUGMODE)
    ROS_INFO_STREAM("innovation vector: \n"<< innovation);

  K_ = ((H_ * P_curr_pred_.transpose() * H_.transpose() + R_.transpose()).ldlt().solve(
      (H_ * P_curr_pred_.transpose()))).transpose();
  if (DEBUGMODE)
    ROS_INFO_STREAM("Kalman gain matrix\n"<< K_);

  P_curr_ = (Eigen::MatrixXd::Identity(kStateSize, kStateSize) - K_ * H_) * P_curr_pred_
      * ((Eigen::MatrixXd::Identity(kStateSize, kStateSize) - K_ * H_).transpose())
      + K_ * R_ * (K_.transpose());
  if (DEBUGMODE)
    ROS_INFO_STREAM("Current state covariance\n"<< P_curr_);

  Eigen::VectorXd delta_state;
  delta_state = K_ * innovation;
  if (DEBUGMODE)
    ROS_INFO_STREAM("K*innovation: delta_state\n"<< delta_state);

  x_curr_ = x_curr_pred_ + delta_state;

  attitude_quat_curr_ = attitude_quat_curr_pred_
      * computeQuaternionFromVectorUSQUE(x_curr_.segment(kAttitudeXYZIndex, 3));

  x_curr_.segment(kAttitudeXYZIndex, 3) << 0, 0, 0;

  return;
}

Eigen::Vector3d UnscentedKalmanFilter::quat2vecUsque(Eigen::Quaterniond input_quat)
{
  if (input_quat.w() == -param_a_) {
    ROS_WARN_STREAM(
        "innovation attitute quaternon has scalar part equal to -1; using its inverse representation");
    input_quat = input_quat.conjugate();
  }
  Eigen::Vector3d output_vect = param_f_ * input_quat.vec() / (param_a_ + input_quat.w());
  return output_vect;
}

Eigen::Quaterniond UnscentedKalmanFilter::computeQuaternionFromVectorUSQUE(
    Eigen::Vector3d vector_of_quaternion_vector_part)
{

  // implementation formula 34a and 34b
  Eigen::Quaterniond quaternion;
  double vect_norm_pwr = pow(vector_of_quaternion_vector_part.norm(), 2);
  double num = -param_a_ * vect_norm_pwr
      + param_f_ * sqrt(param_f_ * param_f_ + (1 - param_a_ * param_a_) * vect_norm_pwr);
  double den = param_f_ * param_f_ + vect_norm_pwr;
  quaternion.w() = num / den;
  quaternion.vec() = 1 / param_f_ * (param_a_ + quaternion.w()) * vector_of_quaternion_vector_part;

  return quaternion;
}

int UnscentedKalmanFilter::hasNaNm(const Eigen::MatrixXd& M)
{
  for (int i = 0; i < M.rows(); i++) {
    for (int j = 0; j < M.cols(); j++) {
      if (std::isnan(M(i, j)))
        return 1;
    }
  }
  return 0;
}

int UnscentedKalmanFilter::hasNaNv(const Eigen::VectorXd& V)
{
  bool flag = 0;
  for (int i = 0; i < V.size(); i++) {
    if (std::isnan(V(i))) {
      return 1;
    }
  }
  return 0;
}

int UnscentedKalmanFilter::hasNaN(const Eigen::Quaterniond& quat)
{
  return std::isnan(quat.vec()(0)) || std::isnan(quat.vec()(1)) || std::isnan(quat.vec()(2))
      || std::isnan(quat.w());
}

void UnscentedKalmanFilter::computeNewForceOffset(){
  force_offset_fsm_.computeAndUseNewOffset();
}

void UnscentedKalmanFilter::useStoredForceOffset(){
  force_offset_fsm_.useParamOffset();
}

void UnscentedKalmanFilter::setForceOffsetParams(const double duration){
  force_offset_fsm_.updateTimerDuration(duration);
}
}  //end of the namespace
