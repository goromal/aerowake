#pragma once

#include <set>
#include <string>

#include <Eigen/Core>

#include <geometry/xform.h>

namespace roscopter
{

namespace ekf
{

namespace meas
{

struct Base
{
    Base();
    virtual ~Base() = default;
    enum
    {
        BASE,
        GNSS,
        IMU,
        BARO,
        RANGE,
        ATT,
        MOCAP,
        ZERO_VEL,
        POS,
#ifdef RELATIVE
        MAG,
#endif
        VEL
    };
    double t;
    int type;
    bool handled;
    std::string Type() const;
};

typedef std::multiset<meas::Base*, std::function<bool(const meas::Base*, const meas::Base*)>> MeasSet;
bool basecmp(const Base *a, const Base *b);

struct Gnss : public Base
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Gnss(double _t, const Vector6d& _z, const Matrix6d& _R);
    Vector6d z;
    Eigen::Map<Eigen::Vector3d> p;
    Eigen::Map<Eigen::Vector3d> v;
    Matrix6d R;
};

struct Imu : public Base
{
    enum
    {
        A = 0,
        W = 3
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Imu(double _t, const Vector6d& _z, const Matrix6d& _R);
    Vector6d z;
    Matrix6d R;
};

struct Baro : public Base
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Baro(double _t, const double& _z, const double& _R, const double& _temp);
    Eigen::Matrix<double, 1, 1> z;
    Eigen::Matrix<double, 1, 1> R;
    double temp;
};

struct Range : public Base
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Range(double _t, const double& _z, const double& _R);
    Eigen::Matrix<double, 1, 1> z;
    Eigen::Matrix<double, 1, 1> R;
};

struct Att : public Base
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Att(double _t, const quat::Quatd& _z, const Eigen::Matrix3d& _R);
    quat::Quatd z;
    Eigen::Matrix3d R;
};

#ifdef RELATIVE
struct Mag : public Base
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Mag(double _t, const Eigen::Vector3d& _z, const Eigen::Matrix3d& _R);
    Eigen::Vector3d z;
    Eigen::Matrix3d R;
};
#endif

struct Pos : public Base
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Pos(double _t, const Eigen::Vector3d& _z, const Eigen::Matrix3d& _R);
    Eigen::Vector3d z;
    Eigen::Matrix3d R;
};

struct Vel : public Base
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vel(double _t, const Eigen::Vector3d& _z, const Eigen::Matrix3d& _R);
    Eigen::Vector3d z;
    Eigen::Matrix3d R;
};

struct Pose : public Base
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Pose(double _t, const xform::Xformd& _z, const Matrix6d& _R);
    xform::Xformd z;
    Matrix6d R;
};

struct ZeroVel: public Base
{
    ZeroVel(double _t);
};

}

}

}

