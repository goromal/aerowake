#include "uav_dynamics/uav_dynamics.h"

namespace uav_dynamics
{

QuadrotorDynamics::QuadrotorDynamics() {}

void QuadrotorDynamics::loadParameters(const Matrix<double, 13, 1> &x0, const double mass, const Matrix3d &inertia,
                                       const Vector3d gravity/*,
                              const double drag_const, const double angular_drag*/)
{
  x_.arr = x0;
  mass_ = mass;
  inertia_matrix_ = inertia;
  inertia_inv_ = inertia_matrix_.inverse();
  gravity_ = gravity;
//  drag_constant_ = drag_const;
//  angular_drag_ = angular_drag;
}

// u = [F(N), Taux(N-m), Tauy, Tauz]
void QuadrotorDynamics::f(const State &x, const Vector4d &ft, ErrorState &dx, const Vector6d &ext_wrench) const
{
//  Eigen::Vector3d v_rel_ = x.v; // Vehicle air velocity
  dx.p = x.q.rota(x.v);
  dx.v = -1.0 * e_z * ft(THRUST) / mass_ + x.q.rotp(gravity_) - x.w.cross(x.v)
          + ext_wrench.block<3,1>(0,0) / mass_; // body frame
  dx.q = x.w;
  dx.w = inertia_inv_ * (ft.segment<3>(TAUX) - x.w.cross(inertia_matrix_ * x.w) /*- angular_drag_ * x.w.cwiseProduct(x.w)*/
                         + ext_wrench.block<3,1>(3,0)); // body frame
}

void QuadrotorDynamics::f(const State &x, const Vector4d &u, ErrorState &dx, Vector6d& imu, const Vector6d &ext_wrench) const
{
    f(x, u, dx, ext_wrench);
    imu.block<3,1>(ACC, 0) = dx.v + x.w.cross(x.v) - x.q.rotp(gravity_);
    imu.block<3,1>(GYRO, 0) = x.w;
}

// u = [F(N), Taux(N-m), Tauy, Tauz]
void QuadrotorDynamics::run(const double dt, const Vector4d &u, const Vector6d &ext_wrench)
{
  // 4th order Runge-Kutta integration
  f(x_, u, k1_, imu_, ext_wrench);

  x2_ = x_;
  x2_ += k1_ * (dt/2.0);
  f(x2_, u, k2_, ext_wrench);

  x3_ = x_;
  x3_ += k2_ * (dt/2.0);
  f(x3_, u, k3_, ext_wrench);

  x4_ = x_;
  x4_ += k3_ * dt;
  f(x4_, u, k4_, ext_wrench);

  dx_ = (k1_ + k2_*2.0 + k3_*2.0 + k4_) * (dt / 6.0);

  // Evolve state
  x_ += dx_;
}

Vector3d QuadrotorDynamics::get_imu_accel() const
{
  return imu_.segment<3>(ACC);
}

Vector3d QuadrotorDynamics::get_imu_gyro() const
{
  return imu_.segment<3>(GYRO);
}

} // end namespace uav_dynamics
