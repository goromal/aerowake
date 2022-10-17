#ifndef UNSCENTED_KALMAN_FILTER_UNSCENTED_KALMAN_FILTER_H
#define UNSCENTED_KALMAN_FILTER_UNSCENTED_KALMAN_FILTER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <geometry_msgs/WrenchStamped.h>

#include "unscented_kalman_filter/force_offset_fsm.h"

namespace unscented_kalman_filter {

class ForceOffsetFsm; // forward declaration for compilation safety reason

class UnscentedKalmanFilter
{
 public:
  UnscentedKalmanFilter(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~UnscentedKalmanFilter();
  void InitializeParameters();
  void SetMotorSpeedMeasurement(const mav_msgs::Actuators& actuators_msg);
  void SetOdometryMeasurement(const mav_msgs::EigenOdometry& odometry);
  void UpdateStatusEstimate();
  geometry_msgs::WrenchStamped getUkfExternalForcesMoments();
  void getUkfEstimatedOdometry(nav_msgs::Odometry* ukf_estimated_odometry);
  geometry_msgs::WrenchStamped getForcesTorqueCovariance();
  void setProcessNoiseCovariance(const Eigen::MatrixXd& Q);
  void setMeasurementNoiseCovariance(const Eigen::MatrixXd& R);
  void setAlphaBetaKappaMrpA(const double alpha, const double beta, const double kappa,
                             const double MRP_a);
  void setForceOffsetParams(const double duration);
  void setSettings(const bool second_order_dynamic, const bool update_step_kalman,
                   const bool impose_quaternion_positivity);
  void computeNewForceOffset();
  void useStoredForceOffset();
  bool DEBUGMODE;

 private:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const size_t kStateSize = 16;
  static const size_t kMeasurementSize = 12;
  static const size_t kSigmaPointsSize = 17;
  static const size_t kSigmaPointsNum = 33;

  // indexes to access in the state/sigma point vector
  static const size_t kPositionIndex = 0;
  static const size_t kVelocityIndex = 3;
  static const size_t kAttitudeXYZIndex = 6;
  static const size_t kAngularRateIndex = 9;
  static const size_t kExternalForcesIndex = 12;
  static const size_t kExternalYawMomentsIndex = 15;
  static const size_t kAttitudeWIndex = 16;
  static const size_t kAttitudeWIndexMeasurement = 12;

  static const int kIntegrationStepQuaternionKinematic = 50;

  // hardcoded parameters
  const double GRAVITY;

  // sensor measurements
  mav_msgs::EigenOdometry odometry_measurement_;
//  Eigen::Matrix<double, 6, 1> rotors_speed_measurement_; // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  int num_rotors_;
  Eigen::MatrixXd rotors_speed_measurement_;
  Eigen::VectorXd z_curr_;

  //filter variables
  bool initialized_parameters_;
  bool filter_active_;

  Eigen::Matrix<double, kStateSize, kStateSize> P_0_;
  Eigen::Matrix<double, kStateSize, kStateSize> P_prev_;
  Eigen::Matrix<double, kStateSize, kStateSize> P_curr_pred_;  //support variable
  Eigen::Matrix<double, kStateSize, kStateSize> P_curr_;

  Eigen::Matrix<double, kStateSize, 1> x_prev_;
  Eigen::Quaterniond attitude_quat_prev_;
  Eigen::Matrix<double, kStateSize, 1> x_curr_pred_;  // support variables
  Eigen::Matrix<double, kSigmaPointsSize, 1> usque_x_curr_pred_;  // usque state after prediction step (full quaterions)
  Eigen::Quaterniond attitude_quat_curr_pred_;
  Eigen::Matrix<double, kStateSize, 1> x_curr_;
  Eigen::Quaterniond attitude_quat_curr_;

  Eigen::Matrix<double, kStateSize, kMeasurementSize> K_; 	//kalman filter gain for update step;

  // Usque (new) other objects
  Eigen::Matrix<double, kSigmaPointsSize, kSigmaPointsNum> sigma_points_;
  Eigen::Matrix<double, kStateSize, kSigmaPointsNum> w_prime_;
  Eigen::Matrix<double, kStateSize, kStateSize> L_;
  Eigen::VectorXd temp_out_;  // vector to propagate sigma points

  // UnscentedTransformation parameters
  Eigen::Quaterniond quaternion_curr_prediction_;  //suppport variable for USQUE approach

  // parameters for the unscented transformation
  Eigen::Matrix<double, 2 * kStateSize + 1, 2 * kStateSize + 1> diagWc_;  // weights for the covariance
  Eigen::Matrix<double, 2 * kStateSize + 1, 2 * kStateSize + 1> diagWm_;  // weights for the mean
  double gamma_; 						// weight factor for the Cholesky decomposition

  // filter parameters
  Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> R_;
  Eigen::Matrix<double, kStateSize, kStateSize> Q_;
  Eigen::Matrix<double, kMeasurementSize, kStateSize> H_;
//  Eigen::Matrix<double, 4, 6> allocation_matrix_; // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<
  Eigen::MatrixXd allocation_matrix_;
  double alpha_, beta_, kappa_;
  double param_a_;  // parameter for USQUE [0,1]
  double param_f_;  // parameter for USQUE -> f = 2*(a+1)

  // Settings
  bool second_order_dynamics_;  //use ro not second order dynamics
  bool advanced_quat_averaging_;
  bool impose_quaternion_positivity_;

  //vehicle parameters
  struct param
  {
    double l;
    double k_n;
    double k_m;
    double I_r;
    double mass;
    Eigen::DiagonalMatrix<double, 3> inertia;
    double k_drag;
    double g;
    double dT;
  } uav_params_;

  // node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // force offset state machine
  force_offset_fsm::ForceOffsetFsm force_offset_fsm_;

  // private methods
  void initializeUnscentedTransformation();
  Eigen::Quaterniond computeQuaternionFromVectorUSQUE(
      Eigen::Vector3d vector_of_quaternion_vector_part);
  void multiStepFullQuaternionIntegration(double end_time, int iterations,
                                          const Eigen::Quaterniond attitude_quaternion,
                                          const Eigen::Vector3d& omega,
                                          Eigen::Quaterniond *integrated_attitude_quaternion);
  void integrateFullQuaternionKinematics3(const Eigen::Vector4d& attitude_quaternion,
                                          const Eigen::Vector3d& omega,
                                          Eigen::Vector4d *integrated_attitude_quaternion,
                                          const double dT);
  void RotationalDynamicsAndThrust(const Eigen::Vector3d& stateInAngvel, const double stateInExtM,
                                   double* thrust, Eigen::Vector3d* angular_acceleration);
  void TranslationalDynamics(const Eigen::Quaterniond& attitude_quaternion,
                             const Eigen::Vector3d& linear_velocity,
                             const Eigen::Vector3d& external_forces, const double thrust,
                             Eigen::Vector3d* linear_acceleration);
  template<typename T> void loadParameter(const ros::NodeHandle& nh,
                                          const std::string parameter_name,
                                          T* paramater_destination);

  // usque functions
  void UsquePredictionStep();
  void UsqueUpdateStep();
  void simulateSystemDynamics(const Eigen::VectorXd& x, Eigen::VectorXd * out);
  void compute_sigma_points(const Eigen::VectorXd &state, const Eigen::Quaterniond &quat_state,
                            const Eigen::MatrixXd &L);
  void compute_sigma_point_mean(const Eigen::MatrixXd &sigma_points);
  Eigen::Quaterniond compute_quaternion_sigma_point_mean(const Eigen::MatrixXd & sigma_points);
  void compute_sigma_point_deltas(const Eigen::MatrixXd & sigma_points,
                                  const Eigen::VectorXd & mean);
  Eigen::Quaterniond update_quaternion_field(const Eigen::Quaterniond& state,
                                             const Eigen::Vector3d& delta);
  Eigen::Vector3d quat2vecUsque(Eigen::Quaterniond input_quat);
  void compute_sigma_point_deltas_mean(const Eigen::MatrixXd& sigma_points_deltas);
  Eigen::Quaterniond computeQuatAvg(const Eigen::MatrixXd& sigma_points,
                                    const Eigen::MatrixXd& weights);
  int hasNaNm(const Eigen::MatrixXd& M);
  int hasNaNv(const Eigen::VectorXd& V);
  int hasNaN(const Eigen::Quaterniond& quat);
};
}
#endif //UNSCENTED_KALMAN_FILTER_UNSCENTED_KALMAN_FILTER_H

