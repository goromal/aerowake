#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "geometry-utils-lib/quat.h"
#include "geometry-utils-lib/xform.h"
#include "math-utils-lib/constants.h"

using namespace Eigen;
using namespace transforms;
using namespace utils_math;

namespace uav_dynamics {

struct ErrorState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum
  {
    SIZE = 12
  };

  Matrix<double, SIZE, 1> arr;
  Map<Vector6d> X;
  Map<Vector3d> p;
  Map<Vector3d> q;
  Map<Vector3d> v;
  Map<Vector3d> w;

  ErrorState() :
    X(arr.data()),
    p(arr.data()),
    q(arr.data()+3),
    v(arr.data()+6),
    w(arr.data()+9)
  {}

  ErrorState(const ErrorState& obj) :
    X(arr.data()),
    p(arr.data()),
    q(arr.data()+3),
    v(arr.data()+6),
    w(arr.data()+9)
  {
    arr = obj.arr;
  }

  ErrorState& operator= (const ErrorState& obj)
  {
    arr = obj.arr;
    return *this;
  }

  ErrorState operator* (const double& s)
  {
    ErrorState out;
    out.arr = s * arr;
    return out;
  }

  ErrorState operator+ (const ErrorState& obj)
  {
    ErrorState out;
    out.arr = obj.arr + arr;
    return out;
  }
};

struct State
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum
  {
    SIZE = 13
  };

  Matrix<double, SIZE, 1> arr;
  Xformd X;
  Map<Vector3d> p;
  Quatd q;
  Map<Vector3d> v;
  Map<Vector3d> w;

  State() :
    X(arr.data()),
    p(arr.data()),
    q(arr.data()+3),
    v(arr.data()+7),
    w(arr.data()+10)
  {
    arr.setZero();
    q = Quatd::Identity();
  }

  State(const State& x) :
    X(arr.data()),
    p(arr.data()),
    q(arr.data()+3),
    v(arr.data()+7),
    w(arr.data()+10)
  {
    arr = x.arr;
  }

  State& operator= (const State& obj)
  {

    arr = obj.arr;
    return *this;
  }

  State operator+(const ErrorState& dx) const
  {
    State xp;
    xp.p = p + dx.p;
    xp.q = q + dx.q;
    xp.v = v + dx.v;
    xp.w = w + dx.w;
    return xp;
  }

  State& operator+=(const ErrorState& dx)
  {
    p = p + dx.p;
    q = q + dx.q;
    v = v + dx.v;
    w = w + dx.w;
    return *this;
  }

  ErrorState operator-(const State& x) const
  {
    ErrorState dx;
    dx.p = p - x.p;
    dx.q = q - x.q;
    dx.v = v - x.v;
    dx.w = w - x.w;
    return dx;
  }
};

struct Input
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum
  {
    SIZE = 4
  };
  Matrix<double, SIZE, 1> arr;
  double& T;
  Map<Vector3d> tau;

  Input() :
    T(*arr.data()),
    tau(arr.data()+3)
  {}
};

struct IMU
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum
  {
    SIZE = 6
  };
  Matrix<double, SIZE, 1> arr;
  Map<Vector3d> acc;
  Map<Vector3d> gyro;

  IMU() :
    acc(arr.data()),
    gyro(arr.data()+3)
  {}
};

// Input indices
enum {
  THRUST = 0,
  TAUX = 1,
  TAUY = 2,
  TAUZ = 3,
  WX = 1,
  WY = 2,
  WZ = 3,
  INPUT_SIZE = 4
};

// IMU indices
enum {
  ACC = 0,
  GYRO = 3
};

//static const double G = 9.80665;

//static const Vector3d gravity_ = [] {
//  Vector3d tmp;
//  tmp << 0, 0, G;
//  return tmp;
//}();

static const Matrix3d M_ = [] {
  Matrix3d tmp;
  tmp << 1, 0, 0, 0, 1, 0, 0, 0, 0;
  return tmp;
}();

class QuadrotorDynamics
{
typedef Matrix<double, 12, 12> Matrix12d;
typedef Matrix<double, 12, 1> Vector12d;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  QuadrotorDynamics();

  void loadParameters(const Matrix<double, 13, 1> &x0, const double mass, const Matrix3d &inertia,
                      const Vector3d gravity/*const double drag_const, const double angular_drag*/);
  void run(const double dt, const Vector4d& u,
           const Vector6d &ext_wrench=(Vector6d() << 0.,0.,0.,0.,0.,0.).finished());

  const State& get_state() const { return x_; }
  State& get_state() { return x_; }
  void set_state(const State& x) { x_ = x; }

  const Xformd& get_global_pose() const { return x_.X; }
//  const double& get_drag() const { return drag_constant_; }
  Vector3d get_imu_accel() const;
  Vector3d get_imu_gyro() const;

private:
  void f(const State& x, const Vector4d& ft, ErrorState& dx, const Vector6d &ext_wrench) const;
  void f(const State& x, const Vector4d& u, ErrorState& dx, Vector6d& imu, const Vector6d &ext_wrench) const;

  // States and RK4 Workspace
  State x_, x2_, x3_, x4_;
  ErrorState dx_, k1_, k2_, k3_, k4_;

  // Parameters
  double mass_;
  Eigen::Matrix3d inertia_matrix_, inertia_inv_;
  Eigen::Vector3d gravity_;
//  double drag_constant_;
//  double angular_drag_;
  Vector6d imu_;
};

} // end namespace uav_dynamics
