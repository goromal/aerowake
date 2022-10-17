#pragma once

#include <Eigen/Core>
#include <geometry/xform.h>

namespace roscopter
{

namespace ekf
{


class ErrorState
{
public:
    enum
    {
        DX = 0,
        DP = 0,
        DQ = 3,
        DV = 6,
        DBA = 9,
        DBG = 12,
        DBB = 15,
        DREF = 16,
#ifdef RELATIVE
        DQREL = 17,
        DSV = 20,
        NDX = 23,
        SIZE = 23
#else
        NDX = 17,
        SIZE = 17
#endif
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, SIZE, 1> arr;
    Eigen::Map<Vector6d> x;
    Eigen::Map<Eigen::Vector3d> p;
    Eigen::Map<Eigen::Vector3d> q;
    Eigen::Map<Eigen::Vector3d> v;
    Eigen::Map<Eigen::Vector3d> ba;
    Eigen::Map<Eigen::Vector3d> bg;
    double& bb; // bias for barometer measurements
    double& ref; // reference global altitude of NED frame
#ifdef RELATIVE
    Eigen::Map<Eigen::Vector3d> qREL; // quaternion from NED to RELATIVE frame
    Eigen::Map<Eigen::Vector3d> sv; // estimated relative base frame velocity
#endif

    ErrorState();
    ErrorState(const ErrorState& obj);
    ErrorState& operator=(const ErrorState& obj);
    ErrorState operator*(const double& s) const;
    ErrorState operator/(const double& s) const;
    ErrorState& operator*=(const double& s);
    ErrorState operator+(const ErrorState& obj) const;
    ErrorState operator-(const ErrorState& obj) const;
    ErrorState operator+(const Eigen::Matrix<double, SIZE, 1>& obj) const;
    ErrorState operator-(const Eigen::Matrix<double, SIZE, 1>& obj) const;
    ErrorState& operator+=(const Eigen::Matrix<double, SIZE, 1>& obj);
    ErrorState& operator-=(const Eigen::Matrix<double, SIZE, 1>& obj);
    ErrorState& operator+=(const ErrorState& obj);
    ErrorState& operator-=(const ErrorState& obj);

    static ErrorState Random()
    {
        ErrorState x;
        x.arr.setRandom();
        return x;
    }

    static ErrorState Zero()
    {
        ErrorState x;
        x.arr.setZero();
        return x;
    }
};

inline std::ostream& operator<< (std::ostream& os, const ErrorState& e)
{
    os << "dp: " << e.p.transpose() << " "
       << "dq: " << e.q.transpose() << " "
       << "dv: " << e.v.transpose() << " "
#ifdef RELATIVE
       << "dqR: " << e.qREL.transpose() << " "
       << "dvs: " << e.sv.transpose();
#else
       ;
#endif
    return os;
}

class State
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum
    {
        T = 0,
        X = 1, // for Xform access (p, q)
        P = 1,
        Q = 4,
        V = 8,
        BA = 11,
        BG = 14,
        BB = 17,
        REF = 18,
#ifdef RELATIVE
        QREL = 19,
        SV = 23,
        NX = 25,
        A = 26,
        W = 29,
#else
        NX = 18, // number of states
        A = 19,
        W = 22,
#endif
        SIZE = 1 + NX + 6
    };
    Eigen::Matrix<double, SIZE, 1> arr;

    Eigen::Map<Vector6d> imu; // IMU measurement at current time
    Eigen::Map<Eigen::Vector3d> a;
    Eigen::Map<Eigen::Vector3d> w;

    double& t; // Time of current state
    xform::Xformd x;
    Eigen::Map<Eigen::Vector3d> p;
    quat::Quatd q;
    Eigen::Map<Eigen::Vector3d> v;
    Eigen::Map<Eigen::Vector3d> ba;
    Eigen::Map<Eigen::Vector3d> bg;
    double& bb; // barometer pressure bias
    double& ref; // reference global altitude of NED frame
#ifdef RELATIVE
    quat::Quatd qREL;
    Eigen::Map<Eigen::Vector3d> sv;
#endif

    State();
    State(const State& other);
    State& operator=(const State& obj);

    static State Random()
    {
        State x;
        x.arr.setRandom();
        x.x = xform::Xformd::Random();
#ifdef RELATIVE
        x.qREL = quat::Quatd::Random();
#endif
        return x;
    }

    static State Identity()
    {
        State out;
        out.x = xform::Xformd::Identity();
        out.v.setZero();
        out.ba.setZero();
        out.bg.setZero();
#ifdef RELATIVE
        out.qREL = quat::Quatd::Identity();
        out.sv.setZero();
#endif
        return out;
    }

    State operator+(const ErrorState &delta) const;
    State operator+(const Eigen::Matrix<double, ErrorState::SIZE, 1> &delta) const;
    State& operator+=(const ErrorState &delta);
    State& operator+=(const Eigen::VectorXd& dx);
    ErrorState operator-(const State &x2) const;
};

inline std::ostream& operator<< (std::ostream& os, const State& s)
{
    os << "p: " << s.p.transpose() << " "
       << "q: " << s.q << " "
       << "v: " << s.v.transpose() << " "
#ifdef RELATIVE
       << "qR: " << s.qREL << " "
       << "vs: " << s.sv.transpose();
#else
       ;
#endif
    return os;
}

typedef Eigen::Matrix<double, ErrorState::SIZE, ErrorState::SIZE> dxMat;
typedef Eigen::Matrix<double, ErrorState::SIZE, 1> dxVec;
typedef Eigen::Matrix<double, ErrorState::SIZE, 6> dxuMat;
typedef Eigen::Matrix<double, 6, 6> duMat;


class StateBuf
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Snapshot
    {
        State x;
        dxMat P;
    };

    std::vector<Snapshot, Eigen::aligned_allocator<Snapshot>> buf; // circular buffer
    int head;
    int tail;
    int size;

    StateBuf(int size);
    State &x();
    const State &x() const;
    dxMat &P();
    const dxMat &P() const;

    Snapshot& next();
    Snapshot& begin();

    void advance();
    bool rewind(double t);
};

}

}
