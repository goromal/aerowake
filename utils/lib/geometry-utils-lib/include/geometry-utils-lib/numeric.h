#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
#include "quat.h"
#include "xform.h"

using namespace Eigen;

namespace transforms {

/// GENERIC OPS
inline void genericSetIdentity(Quatd &val)
{
    val = Quatd::Identity();
}
inline void genericSetIdentity(Xformd &val)
{
    val = Xformd::Identity();
}

/// MANIFOLD DIFFERENTIATOR
template<typename Derived, typename T>
class Differentiator
{
private:
    double sigma_;
    bool initialized_;
    Derived deriv_curr_;
    Derived deriv_prev_;
    T val_prev_;
public:
    Differentiator(void) : sigma_(0.05)
    {
        reset();
    }
    Differentiator(const double sigma) : sigma_(sigma)
    {
        reset();
    }
    void init(const double sigma)
    {
        sigma_ = sigma;
        reset();
    }
    void reset()
    {
        deriv_curr_.setZero();
        deriv_prev_.setZero();
        genericSetIdentity(val_prev_);
        initialized_ = false;
    }
    Derived calculate(const T &val, const double Ts)
    {
        if (initialized_)
        {
            deriv_curr_ = (2 * sigma_ - Ts) / (2 * sigma_ + Ts) * deriv_prev_ +
                          2 / (2 * sigma_ + Ts) * (val - val_prev_);
            deriv_prev_ = deriv_curr_;
            val_prev_ = val;
        }
        else
        {
            deriv_curr_.setZero();
            deriv_prev_ = deriv_curr_;
            val_prev_ = val;
            initialized_ = true;
        }
        return deriv_curr_;
    }
};

typedef Differentiator<Matrix<double, 3, 1>, Quatd>  QuatdDifferentiator;
typedef Differentiator<Matrix<double, 6, 1>, Xformd> XformdDifferentiator;

/// MANIFOLD INTERPOLATOR
/// TODO

} // end namespace transforms
