#include "data_structure.hpp"

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <gmock/gmock-matchers.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sstream>

#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/KroneckerProduct>


using namespace ::testing;


double tol = 1e-3;
double prec = 3;


// ====================
// Test fixture
// ====================

class StateSpaceTest : public ::testing::Test {
 protected:
    void SetUp() override {
        // Sample system: mass-spring-damper or simple 2nd order
        A_ = (Eigen::MatrixXd(2, 2) << 0, 1, -2, -3).finished();
        B_ = (Eigen::VectorXd(2) << 0, 1).finished();
    }

    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
};


class OutputSpaceTest : public ::testing::Test {
 protected:
    void SetUp() override {
        // Sample system: mass-spring-damper or simple 2nd order
        A_ = (Eigen::MatrixXd(2, 2) << 0, 1, -2, -3).finished();
        B_ = (Eigen::VectorXd(2) << 0, 1).finished();
        C_ = (Eigen::RowVectorXd(2) << 1, 0).finished();
        D_ = Eigen::MatrixXd::Zero(1, 1);
    }

    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd C_;
    Eigen::MatrixXd D_;
};



// ====================
// Test StateSpace
// ====================

TEST_F(StateSpaceTest, State_SpaceConstructor_AB_Valid) {
    StateSpace ss(A_, B_);
    EXPECT_EQ(ss.nx, 2);
    EXPECT_EQ(ss.nu, 1);
    EXPECT_EQ(ss.ny, 0);
    EXPECT_TRUE(ss.C.isZero());
    EXPECT_TRUE(ss.D.isZero());
}

TEST_F(StateSpaceTest, State_SpaceConstructor_AB_InvalidDims) {
    Eigen::MatrixXd bad_A = Eigen::MatrixXd::Identity(3, 2);
    EXPECT_THROW(StateSpace(bad_A, B_), std::invalid_argument);

    Eigen::MatrixXd bad_B(3, 1);
    EXPECT_THROW(StateSpace(A_, bad_B), std::invalid_argument);
}

TEST_F(OutputSpaceTest, State_SpaceConstructor_ABC_Valid) {
    StateSpace ss(A_, B_, C_);
    EXPECT_EQ(ss.nx, 2);
    EXPECT_EQ(ss.nu, 1);
    EXPECT_EQ(ss.ny, 1);
    EXPECT_EQ(ss.C, C_);
    EXPECT_EQ(ss.D, Eigen::MatrixXd::Zero(ss.ny, ss.nu));
}

TEST_F(OutputSpaceTest, State_SpaceConstructor_ABCD_Valid) {
    StateSpace ss(A_, B_, C_, D_);
    EXPECT_EQ(ss.nx, 2);
    EXPECT_EQ(ss.nu, 1);
    EXPECT_EQ(ss.ny, 1);
    EXPECT_EQ(ss.C, C_);
    EXPECT_EQ(ss.D, D_);
}

TEST_F(OutputSpaceTest, State_SpaceConstructor_ABCD_InvalidDims) {
    Eigen::MatrixXd bad_C(1, 3);
    EXPECT_THROW(StateSpace(A_, B_, bad_C, D_), std::invalid_argument);

    Eigen::MatrixXd bad_D(1, 2);
    EXPECT_THROW(StateSpace(A_, B_, C_, bad_D), std::invalid_argument);
}


