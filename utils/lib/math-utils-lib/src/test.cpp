#include "math-utils-lib/control.h"
#include "math-utils-lib/modeling.h"
#include "math-utils-lib/nano_ad.h"
#include "math-utils-lib/constants.h"
#include "math-utils-lib/generic.h"
#include "math-utils-lib/tools.h"

#include "gtest/gtest.h"

#define MATRIX_CLOSE(m1, m2, tol) {\
    for (int row = 0; row < (m1).rows(); row++ ) \
{ \
    for (int col = 0; col < (m1).cols(); col++) \
{ \
    EXPECT_NEAR((m1)(row, col), (m2)(row, col), tol); \
    } \
    } \
    }

TEST(RK4, Type_Test)
{
  // Will return true...
}
