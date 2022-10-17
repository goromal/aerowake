#include "logging-utils-lib/logging.h"
#include "logging-utils-lib/yaml.h"
#include "logging-utils-lib/filesystem.h"

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

TEST(matrixToLog, Full_Test)
{

}
