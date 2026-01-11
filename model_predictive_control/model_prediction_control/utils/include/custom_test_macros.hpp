#ifndef CUSTOM_TEST_MACROS_HPP
#define CUSTOM_TEST_MACROS_HPP
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <gmock/gmock-matchers.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sstream>


using namespace ::testing;


// ====================
// Helper macros
// ====================
template <typename DerivedA, typename DerivedB>
::testing::AssertionResult AssertMatricesNear(
    const Eigen::MatrixBase<DerivedA>& A_in,
    const Eigen::MatrixBase<DerivedB>& B_in,
    double tol,
    int precision = 3)
{
    // Make concrete dense matrices (safe to call .rows(), .cols(), print, etc.)
    const Eigen::MatrixXd A = A_in.template eval();
    const Eigen::MatrixXd B = B_in.template eval();

    if (A.rows() != B.rows() || A.cols() != B.cols()) {
        std::ostringstream oss;
        oss << "Size mismatch: Actual is " << A.rows() << "x" << A.cols()
            << ", Expected is " << B.rows() << "x" << B.cols();
        return ::testing::AssertionFailure() << oss.str();
    }

    // difference and max absolute error
    Eigen::MatrixXd diff = (A - B).cwiseAbs();
    double maxErr = diff.size() ? diff.maxCoeff() : 0.0;

    if (maxErr <= tol) {
        return ::testing::AssertionSuccess();
    }

    // Build human-friendly message with requested precision
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    oss << "Max error: " << maxErr << " (tol = " << tol << ")\n\n";
    oss << "Actual:\n" << A << "\n\n";
    oss << "Expected:\n" << B << "\n\n";

    return ::testing::AssertionFailure() << oss.str();
}

// Convenience macros (easy to use in tests)
#define EXPECT_MATRIX_NEAR(A, B, tol, precision) \
    EXPECT_TRUE(AssertMatricesNear((A), (B), (tol), (precision)))

#define ASSERT_MATRIX_NEAR(A, B, tol, precision) \
    ASSERT_TRUE(AssertMatricesNear((A), (B), (tol), (precision)))


#endif // CUSTOM_TEST_MACROS_HPP