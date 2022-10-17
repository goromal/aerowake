#pragma once

#include "modeling.h"

using namespace Eigen;
using namespace utils_math;

namespace control {

// Must be with double, float, or Eigen type!
// Eigen type could be like a vector of positions or something...
template<typename T>
class PID
{
private:
    template<typename Derived>
    Eigen::MatrixBase<Derived> antiWindup(const Eigen::MatrixBase<Derived> &usat,
                                          const Eigen::MatrixBase<Derived> &unsat,
                                          const Eigen::MatrixBase<Derived> &P,
                                          const Eigen::MatrixBase<Derived> &error_int,
                                          const Eigen::MatrixBase<Derived> &D,
                                          const Eigen::MatrixBase<Derived> &ki)
    {
        Eigen::MatrixBase<Derived> aW;
        for (int i = 0; i < usat.rows(); i++)
        {
            for (int j = 0; j < usat.cols(); j++)
            {
                if (usat(i, j) != unsat(i, j) && ki(i, j) > 0.0)
                    aW(i, j) = (usat(i, j) - P(i, j) + D(i, j)) / ki(i, j);
                else
                    aW(i, j) = error_int(i, j);
            }
        }
        return aW;
    }
    double antiWindup(const double &usat, const double &unsat, const double &P,
                      const double &error_int, const double &D, const double &ki)
    {
        double aW;
        if (usat != unsat && ki > 0.0)
            aW = (usat - P + D) / ki;
        else
            aW = error_int;
        return aW;
    }
    float antiWindup(const float &usat, const float &unsat, const float &P,
                      const float &error_int, const float &D, const float &ki)
    {
        float aW;
        if (usat != unsat && ki > 0.0)
            aW = (usat - P + D) / ki;
        else
            aW = error_int;
        return aW;
    }
    T kp_;
    T ki_;
    T kd_;
    T max_;
    modeling::Integrator<T> integrator_;
    modeling::Differentiator<T> differentiator_;
public:
    PID()
    {
        genericSetZero(kp_);
        genericSetZero(ki_);
        genericSetZero(kd_);
        genericSetZero(max_);
    }
    void init(T kp, T ki, T kd, T max)
    {
        init(kp, ki, kd, max, 0.05);
    }
    void init(T kp, T ki, T kd, T max, double sigma)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        max_ = max;
        differentiator_.init(sigma);
    }
    void reset()
    {
        T integralVal;
        genericSetZero(integralVal);
        integrator_.init(integralVal);
        integrator_.reset();
        differentiator_.reset();
    }
    T run(const double dt, T x, T x_c, bool update_integrator)
    {
        T xdot;
        if (dt > 0.0001)
            xdot = differentiator_.calculate(x, dt);
        else
            genericSetZero(xdot);
        return run(dt, x, x_c, update_integrator, xdot);
    }
    T run(const double dt, T x, T x_c, bool update_integrator, T xdot)
    {
        T error = x_c - x;
        T error_int = integrator_.calculate(error, dt);

        T P = genericElementwiseMultiply(kp_, error);
        T I = genericElementwiseMultiply(ki_, error_int);
        T D = genericElementwiseMultiply(kd_, xdot);

        T u = P + I - D;

        T u_sat = genericSat(u, max_);
        integrator_.setIntegral(antiWindup(u_sat, u, P, error_int, D, ki_));

        return u_sat;
    }
};

} // end namespace control
