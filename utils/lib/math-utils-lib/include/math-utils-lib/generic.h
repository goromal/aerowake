#pragma once

#include <Eigen/Core>

using namespace Eigen;

namespace utils_math
{

typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;

typedef Eigen::Matrix<double, 5, 5> Matrix5d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;
typedef Eigen::Matrix<double, 8, 8> Matrix8d;
typedef Eigen::Matrix<double, 9, 9> Matrix9d;

/// SET ZERO
template<typename Derived>
inline void genericSetZero(Eigen::MatrixBase<Derived> &mat)
{
    mat.setZero();
}
inline void genericSetZero(double &val)
{
    val = 0.0;
}
inline void genericSetZero(float &val)
{
    val = 0.0;
}

/// ELEMENTWISE MULTIPLY
template<typename Derived>
inline Eigen::MatrixBase<Derived> genericElementwiseMultiply(const Eigen::MatrixBase<Derived> &A,
                                                      const Eigen::MatrixBase<Derived> &B)
{
    return A.cwiseProduct(B);
}
inline double genericElementwiseMultiply(const double &a, const double &b)
{
    return a * b;
}
inline float genericElementwiseMultiply(const float &a, const float &b)
{
    return a * b;
}

/// ELEMENTWISE DIVIDE
template<typename Derived>
inline Eigen::MatrixBase<Derived> genericElementwiseDivide(const Eigen::MatrixBase<Derived> &A,
                                                    const Eigen::MatrixBase<Derived> &B)
{
  return A.cwiseQuotient(B);
}
inline double genericElementwiseDivide(const double &a, const double &b)
{
    return a / b;
}
inline float genericElementwiseDivide(const float &a, const float &b)
{
    return a / b;
}

/// SATURATION
template<typename Derived>
inline Eigen::MatrixBase<Derived> genericSat(const Eigen::MatrixBase<Derived> &unsat,
                                      const Eigen::MatrixBase<Derived> &max)
{
    Eigen::MatrixBase<Derived> sat;
    for (int i = 0; i < unsat.rows(); i++)
    {
        for (int j = 0; j < unsat.cols(); j++)
        {
            sat(i, j) = (unsat(i, j) > max(i, j)) ? max(i, j) :
                        (unsat(i, j) < -1.0 * max(i, j)) ? -1.0 * max(i, j) :
                        unsat(i, j);
        }
    }
    return sat;
}
inline double genericSat(const double &unsat, const double &max)
{
    return (unsat > max) ? max : (unsat < -1.0 * max) ? -1.0 * max : unsat;
}
inline float genericSat(const float &unsat, const float &max)
{
    return (unsat > max) ? max : (unsat < -1.0 * max) ? -1.0 * max : unsat;
}

/// NORM
template<typename Derived>
inline double genericNorm(const Eigen::MatrixBase<Derived> &mat)
{
    return mat.norm();
}
inline double genericNorm(const double &val)
{
    return fabs(val);
}
inline float genericNorm(const float &val)
{
    return fabs(val);
}

} // end namespace utils_math
