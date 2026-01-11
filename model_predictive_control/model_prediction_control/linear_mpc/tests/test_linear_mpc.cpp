#include "linear_mpc.hpp"

#include "custom_test_macros.hpp"

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <gmock/gmock-matchers.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sstream>

#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/KroneckerProduct>


using namespace ::testing;
using namespace mpc;


double tol = 1e-3;
double prec = 3;



// ====================
// Helper function
// ====================

// Mock for IQpSolver
class MockQpSolver : public IQpSolver {
public:
    // Exact same signature as base class
    MOCK_METHOD(Eigen::VectorXd, solve,
        (const Eigen::MatrixXd& H,
         const Eigen::VectorXd& f,
         const Eigen::MatrixXd& G_ineq,
         const Eigen::VectorXd& E_ineq,
         const Eigen::MatrixXd& G_eq,
         const Eigen::VectorXd& E_eq),
        (override));
};


// ====================
// Test fixture
// ====================

class LinearMpcStateTest : public ::testing::Test {
 protected:
    void SetUp() override {
        // Sample system: mass-spring-damper or simple 2nd order
        A_ = (Eigen::MatrixXd(2, 2) << 0, 1, -2, -3).finished();
        B_ = (Eigen::VectorXd(2) << 0, 1).finished();

        mpc_data_.discretization = "Euler";
        mpc_data_.Ts = 0.1;
        mpc_data_.Np = 10;
        mpc_data_.Nc = 5;
        mpc_data_.Q = Eigen::MatrixXd::Identity(2, 2) * 10;
        mpc_data_.R = Eigen::MatrixXd::Identity(1, 1) * 1;
        mpc_data_.S = Eigen::MatrixXd::Identity(1, 1);

        // Bounds (arbitrary for testing)
        mpc_data_.x_min = Eigen::VectorXd::Constant(2, -10);
        mpc_data_.x_max = Eigen::VectorXd::Constant(2, 10);
        mpc_data_.u_min = Eigen::VectorXd::Constant(1, -5);
        mpc_data_.u_max = Eigen::VectorXd::Constant(1, 5);
        mpc_data_.du_min = Eigen::VectorXd::Constant(1, -1);
        mpc_data_.du_max = Eigen::VectorXd::Constant(1, 1);

        mpc_data_mod_ = mpc_data_;

        mock_qp_solver = std::make_shared<MockQpSolver>();
    }

    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    MpcData mpc_data_;
    MpcData mpc_data_mod_;
    std::shared_ptr<MockQpSolver> mock_qp_solver;
};


class LinearMpcOutputTest : public ::testing::Test {
 protected:
    void SetUp() override {
        // Sample system: mass-spring-damper or simple 2nd order
        A_ = (Eigen::MatrixXd(2, 2) << 0, 1, -2, -3).finished();
        B_ = (Eigen::VectorXd(2) << 0, 1).finished();
        C_ = (Eigen::RowVectorXd(2) << 1, 0).finished();
        D_ = Eigen::MatrixXd::Zero(1, 1);

        mpc_data_.discretization = "Euler";
        mpc_data_.Ts = 0.1;
        mpc_data_.Np = 10;
        mpc_data_.Nc = 5;
        mpc_data_.Q = Eigen::MatrixXd::Identity(1, 1) * 10;
        mpc_data_.R = Eigen::MatrixXd::Identity(1, 1) * 1;
        mpc_data_.S = Eigen::MatrixXd::Identity(1, 1);

        // Bounds (arbitrary for testing)
        mpc_data_.y_min = Eigen::VectorXd::Constant(1, -10);
        mpc_data_.y_max = Eigen::VectorXd::Constant(1, 10);
        mpc_data_.u_min = Eigen::VectorXd::Constant(1, -5);
        mpc_data_.u_max = Eigen::VectorXd::Constant(1, 5);
        mpc_data_.du_min = Eigen::VectorXd::Constant(1, -1);
        mpc_data_.du_max = Eigen::VectorXd::Constant(1, 1);

        mpc_data_mod_ = mpc_data_;

        mock_qp_solver = std::make_shared<MockQpSolver>();
    }

    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd C_;
    Eigen::MatrixXd D_;
    MpcData mpc_data_;
    MpcData mpc_data_mod_;
    std::shared_ptr<MockQpSolver> mock_qp_solver;
};


class LinearMpcTestAccessor : public mpc::LinearMpc {
public:
    using mpc::LinearMpc::LinearMpc;   // inherit constructor
    using mpc::LinearMpc::prediction;  // expose protected method
    using mpc::LinearMpc::optimization;  // expose protected method
};



// ====================
// Test LinearMpc State
// ====================

// Constructor
TEST_F(LinearMpcStateTest, State_Constructor_Weights_Fail) {
    StateSpace ss(A_, B_);

    mpc_data_mod_.Q = Eigen::MatrixXd::Identity(2, 3);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.R = Eigen::MatrixXd::Identity(2, 1);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.S = Eigen::MatrixXd::Identity(2, 1);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);
}


TEST_F(LinearMpcStateTest, State_Constructor_Constraints_Fail) {
    StateSpace ss(A_, B_);

    mpc_data_mod_.x_min = Eigen::VectorXd::Constant(5, -10);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.x_max = Eigen::VectorXd::Constant(6, 10);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.u_max = Eigen::VectorXd::Constant(2, 10);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.u_max = Eigen::VectorXd::Constant(3, 10);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.du_max = Eigen::VectorXd::Constant(4, 10);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.du_max = Eigen::VectorXd::Constant(5, 10);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);
}


TEST_F(LinearMpcStateTest, State_Constructor_Success) {
    StateSpace ss(A_, B_);

    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    // Expected prediction matrices
    Eigen::MatrixXd expected_Ad(mpc.nx(), mpc.nx());
    expected_Ad << 1.000, 0.100,
                   -0.200, 0.700;

    Eigen::MatrixXd expected_Bd(mpc.nx(), mpc.nu());
    expected_Bd << 0.000, 0.100;

    // Expected optimization matrices
    Eigen::MatrixXd expected_Qp(mpc.Np() * mpc.nx(), mpc.Np() * mpc.nx());
    expected_Qp = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(mpc.Np(), mpc.Np()), mpc.Q()).eval();

    Eigen::MatrixXd expected_Rp(mpc.Nc() * mpc.nu(), mpc.Nc() * mpc.nu());
    expected_Rp = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(mpc.Nc(), mpc.Nc()), mpc.R()).eval();

    Eigen::MatrixXd expected_Sp(mpc.Nc() * mpc.nu(), mpc.Nc() * mpc.nu());
    expected_Sp = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(mpc.Nc(), mpc.Nc()), mpc.S()).eval();

    Eigen::VectorXd expected_X_min(mpc.Np() * mpc.nx());
    expected_X_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(mpc.Np()), mpc.x_min()).eval();

    Eigen::VectorXd expected_X_max(mpc.Np() * mpc.nx());
    expected_X_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(mpc.Np()), mpc.x_max()).eval();

    Eigen::VectorXd expected_U_min(mpc.Nc() * mpc.nu());
    expected_U_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(mpc.Nc()), mpc.u_min()).eval();

    Eigen::VectorXd expected_U_max(mpc.Nc() * mpc.nu());
    expected_U_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(mpc.Nc()), mpc.u_max()).eval();

    Eigen::VectorXd expected_dU_min(mpc.Nc() * mpc.nu());
    expected_dU_min = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(mpc.Nc()), mpc.du_min()).eval();

    Eigen::VectorXd expected_dU_max(mpc.Nc() * mpc.nu());
    expected_dU_max = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(mpc.Nc()), mpc.du_max()).eval();

    // EXPECT_TRUE(mpc.ssd().A.isApprox(expected_Ad, tol));
    EXPECT_MATRIX_NEAR(mpc.ssd().A, expected_Ad, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.ssd().B, expected_Bd, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.Qp(), expected_Qp, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.Rp(), expected_Rp, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.Sp(), expected_Sp, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.X_min(), expected_X_min, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.X_max(), expected_X_max, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.U_min(), expected_U_min, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.U_max(), expected_U_max, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.dU_min(), expected_dU_min, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.dU_max(), expected_dU_max, tol, prec);
}


TEST_F(LinearMpcStateTest, State_Prediction_Zero_State_Zero_Input_Success) {
    StateSpace ss(A_, B_);
    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(mpc.nx());
    Eigen::VectorXd U = Eigen::VectorXd::Zero(mpc.Nc() * mpc.nu());

    Eigen::MatrixXd X = mpc.prediction(x0, U);

    // Expected dimension: X shape Np*nx
    EXPECT_EQ(X.size(), mpc.Np() * mpc.nx());

    // Expected value
    EXPECT_TRUE(X.isZero());
}


TEST_F(LinearMpcStateTest, State_Prediction_Zero_Input_Success) {
    StateSpace ss(A_, B_);
    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    Eigen::VectorXd x0(mpc.nx());
    x0 << 1, 2;
    Eigen::VectorXd U = Eigen::VectorXd::Zero(mpc.Nc() * mpc.nu());

    Eigen::MatrixXd X = mpc.prediction(x0, U);

    Eigen::VectorXd expected_X(mpc.Np() * mpc.nx());
    expected_X << 1.200, 1.200, 1.320, 0.600, 1.380, 0.156, 1.396, -0.167, 1.379, -0.396,
                  1.339, -0.553, 1.284, -0.655, 1.219, -0.715, 1.147, -0.744, 1.073, -0.750;

    // Expected dimension: X shape Np*nx
    EXPECT_EQ(X.size(), mpc.Np() * mpc.nx());

    // Expected value
    EXPECT_MATRIX_NEAR(X, expected_X, tol, prec);
}


TEST_F(LinearMpcStateTest, State_Prediction_Zero_State_Success) {
    StateSpace ss(A_, B_);
    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(mpc.nx());

    // Non-zero constant input over the control horizon (repeated for Nc steps)
    Eigen::VectorXd U(mpc.nu() * mpc.Nc());
    U.setConstant(1.0);  // Apply u = 1 for all Nc steps

    Eigen::MatrixXd X = mpc.prediction(x0, U);

    // Expected dimension: stacked states over prediction horizon
    EXPECT_EQ(X.rows(), mpc.Np() * mpc.nx());
    EXPECT_EQ(X.cols(), 1);

    Eigen::VectorXd expected_X(mpc.Np() * mpc.nx());
    expected_X <<
        0.000, 0.100,
        0.010, 0.170,
        0.027, 0.217,
        0.049, 0.246,
        0.073, 0.263,
        0.110, 0.239,
        0.134, 0.146,
        0.148, 0.075,
        0.156, 0.023,
        0.158, -0.015;

    EXPECT_MATRIX_NEAR(X, expected_X, tol, prec);
}


TEST_F(LinearMpcStateTest, State_Prediction_Success) {
    StateSpace ss(A_, B_);
    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    // Non-zero initial state
    Eigen::VectorXd x0(mpc.nx());
    x0 << 1.0, 2.0;

    // Non-zero input sequence: ramp from 0 to 1 over Nc steps
    Eigen::VectorXd U(mpc.Nc() * mpc.nu());
    U << 1.0, 0.8, 0.4, -0.2, -0.6;

    Eigen::MatrixXd X = mpc.prediction(x0, U);

    // Expected dimension
    EXPECT_EQ(X.size(), mpc.Np() * mpc.nx());

    // Expected trajectory: free response (from x0) + forced response (from u)
    // Computed via simulation or matrix multiplication Ap*x0 + Bp*u
    Eigen::VectorXd expected_X(mpc.Np() * mpc.nx());
    expected_X <<
        1.200, 1.300,
        1.330, 0.750,
        1.405, 0.299,
        1.435, -0.092,
        1.426, -0.411,
        1.379, -0.615,
        1.317, -0.706,
        1.246, -0.758,
        1.171, -0.780,
        1.093, -0.780;

    EXPECT_MATRIX_NEAR(X, expected_X, tol, prec);
}


TEST_F(LinearMpcStateTest, State_Unc_Opt_Zero_State_Zero_Ref_Success) {
StateSpace ss(A_, B_);
    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(mpc.nx());
    Eigen::VectorXd ref = Eigen::VectorXd::Zero(mpc.nx());

    // Since everything is zero, unconstrained optimum is U = 0
    Eigen::VectorXd expected_U_opt(mpc.Nc() * mpc.nu());
    expected_U_opt.setZero();

    EXPECT_CALL(*mock_qp_solver, solve(_, _, _, _, _, _))
        .WillOnce(Return(expected_U_opt));

    Eigen::VectorXd u = mpc.optimization(x0, ref);

    EXPECT_EQ(u.size(), mpc.nu() * mpc.Nc());
    EXPECT_TRUE(u.isZero(1e-10)) << "Optimal input should be zero when x0=0 and ref=0";
}


TEST_F(LinearMpcStateTest, State_Unc_Opt_Tracking_Constant_Ref_Success) {
    StateSpace ss(A_, B_);
    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(mpc.nx());

    // Constant reference: track state [1.0, 0.0] over entire horizon
    Eigen::VectorXd ref(mpc.nx());
    ref << 1.0, 0.0;
    Eigen::VectorXd REF = ref.replicate(mpc.Np(), 1);

    // Manually compute the unconstrained optimal U
    // Standard QP: min 0.5 U^T H U + g^T U
    // H = Bp^T Qp Bp + Rp
    // g = Bp^T Qp (Ap x0 - ref)   → note: sometimes written as 2*g in some formulations
    Eigen::MatrixXd H = mpc.Bp().transpose() * mpc.Qp() * mpc.Bp() + mpc.Rp();
    Eigen::VectorXd prediction_free = mpc.Ap() * x0;
    Eigen::VectorXd error = prediction_free - REF;
    Eigen::VectorXd g = mpc.Bp().transpose() * mpc.Qp() * error;

    // Solve unconstrained: expected_U_opt = -H^{-1} g
    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
    ASSERT_EQ(ldlt.info(), Eigen::Success) << "Hessian not positive definite";

    Eigen::VectorXd expected_U_opt = ldlt.solve(-g);

    // Mock solver to return this analytical solution
    EXPECT_CALL(*mock_qp_solver, solve(_, _, _, _, _, _))
        .WillOnce(Return(expected_U_opt));

    Eigen::VectorXd U = mpc.optimization(x0, ref);

    // Check that the first applied control input matches the first block of expected_U_opt
    EXPECT_EQ(U.size(), mpc.nu() * mpc.Nc());
    EXPECT_TRUE(U.isApprox(expected_U_opt.head(mpc.nu() * mpc.Nc()), 1e-8)) << "First applied input does not match analytical unconstrained solution";
}



// =====================
// Test LinearMpc Output
// =====================

// Constructor
TEST_F(LinearMpcOutputTest, Output_Constructor_Weights_Fail) {
    StateSpace ss(A_, B_, C_, D_);

    mpc_data_mod_.Q = Eigen::MatrixXd::Identity(2, 3);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.R = Eigen::MatrixXd::Identity(2, 1);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.S = Eigen::MatrixXd::Identity(2, 1);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);
}


TEST_F(LinearMpcOutputTest, Output_Constructor_constraints_Fail) {
    StateSpace ss(A_, B_, C_, D_);

    mpc_data_mod_.y_min = Eigen::VectorXd::Constant(3, -10);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.y_max = Eigen::VectorXd::Constant(3, 10);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.u_max = Eigen::VectorXd::Constant(2, 10);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.u_max = Eigen::VectorXd::Constant(3, 10);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.du_max = Eigen::VectorXd::Constant(4, 10);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);

    mpc_data_mod_ = mpc_data_;

    mpc_data_mod_.du_max = Eigen::VectorXd::Constant(5, 10);
    EXPECT_THROW(LinearMpcTestAccessor mpc(ss, mpc_data_mod_, mock_qp_solver), std::invalid_argument);
}


TEST_F(LinearMpcOutputTest, Output_Constructor_Success) {
    StateSpace ss(A_, B_, C_, D_);

    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    EXPECT_EQ(mpc.Bp().rows(), mpc.Np() * mpc.nx());
    EXPECT_EQ(mpc.Bp().cols(), mpc.Nc() * mpc.nu());
    if (mpc.ny() > 0) {
        EXPECT_EQ(mpc.Cp().rows(), mpc.Np() * mpc.ny());
        EXPECT_EQ(mpc.Dp().rows(), mpc.Np() * mpc.ny());
    }

    // Discretized system: A_d, B_d, C_d = C, D_d = D
    Eigen::MatrixXd expected_Ad(mpc.nx(), mpc.nx());
    expected_Ad << 1.000, 0.100,
                   -0.200, 0.700;

    Eigen::MatrixXd expected_Bd(mpc.nx(), mpc.nu());
    expected_Bd << 0.000, 0.100;

    Eigen::MatrixXd expected_Cd = ss.C;
    Eigen::MatrixXd expected_Dd = ss.D;

    // Qp now blocks Q (ny × ny) over Np
    Eigen::MatrixXd expected_Qp(mpc.Np() * mpc.ny(), mpc.Np() * mpc.ny());
    expected_Qp = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(mpc.Np(), mpc.Np()), mpc.Q()).eval();

    Eigen::MatrixXd expected_Rp(mpc.Nc() * mpc.nu(), mpc.Nc() * mpc.nu());
    expected_Rp = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(mpc.Nc(), mpc.Nc()), mpc.R()).eval();

    Eigen::MatrixXd expected_Sp(mpc.Nc() * mpc.nu(), mpc.Nc() * mpc.nu());
    expected_Sp = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(mpc.Nc(), mpc.Nc()), mpc.S()).eval();

    // Output constraints stacked over Np
    Eigen::VectorXd expected_Y_min(mpc.Np() * mpc.ny());
    expected_Y_min = mpc_data_.y_min.replicate(mpc.Np(), 1);

    Eigen::VectorXd expected_Y_max(mpc.Np() * mpc.ny());
    expected_Y_max = mpc_data_.y_max.replicate(mpc.Np(), 1);

    // Input and rate constraints (same as state case)
    Eigen::VectorXd expected_U_min(mpc.Nc() * mpc.nu());
    expected_U_min = mpc_data_.u_min.replicate(mpc.Nc(), 1);

    Eigen::VectorXd expected_U_max(mpc.Nc() * mpc.nu());
    expected_U_max = mpc_data_.u_max.replicate(mpc.Nc(), 1);

    Eigen::VectorXd expected_dU_min(mpc.Nc() * mpc.nu());
    expected_dU_min = mpc_data_.du_min.replicate(mpc.Nc(), 1);

    Eigen::VectorXd expected_dU_max(mpc.Nc() * mpc.nu());
    expected_dU_max = mpc_data_.du_max.replicate(mpc.Nc(), 1);

    // Checks
    EXPECT_MATRIX_NEAR(mpc.ssd().A, expected_Ad, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.ssd().B, expected_Bd, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.ssd().C, expected_Cd, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.ssd().D, expected_Dd, tol, prec);

    EXPECT_MATRIX_NEAR(mpc.Qp(), expected_Qp, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.Rp(), expected_Rp, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.Sp(), expected_Sp, tol, prec);

    EXPECT_MATRIX_NEAR(mpc.Y_min(), expected_Y_min, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.Y_max(), expected_Y_max, tol, prec);

    EXPECT_MATRIX_NEAR(mpc.U_min(), expected_U_min, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.U_max(), expected_U_max, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.dU_min(), expected_dU_min, tol, prec);
    EXPECT_MATRIX_NEAR(mpc.dU_max(), expected_dU_max, tol, prec);
}


TEST_F(LinearMpcOutputTest, Output_Prediction_zero_state_zero_input_Success) {
    StateSpace ss(A_, B_, C_, D_);
    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(mpc.nx());
    Eigen::VectorXd U = Eigen::VectorXd::Zero(mpc.Nc() * mpc.nu());

    Eigen::MatrixXd Y = mpc.prediction(x0, U);

    EXPECT_EQ(Y.size(), mpc.Np() * mpc.ny());
    EXPECT_TRUE(Y.isZero());  // y = C*0 + D*0 = 0
}


TEST_F(LinearMpcOutputTest, Output_Prediction_zero_input_Success) {
    StateSpace ss(A_, B_, C_, D_);
    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    Eigen::VectorXd x0(mpc.nx());
    x0 << 1.0, 2.0;
    Eigen::VectorXd U = Eigen::VectorXd::Zero(mpc.Nc() * mpc.nu());

    Eigen::MatrixXd Y = mpc.prediction(x0, U);

    // Example assuming C = [1 0], D = 0 → y_k = x1_k (position)
    // Use values from state prediction's first component
    Eigen::VectorXd expected_Y(mpc.Np() * mpc.ny());
    expected_Y << 1.200, 1.320, 1.380, 1.396, 1.379,
                  1.339, 1.284, 1.219, 1.147, 1.073;

    EXPECT_EQ(Y.size(), mpc.Np() * mpc.ny());
    EXPECT_MATRIX_NEAR(Y, expected_Y, tol, prec);
}


TEST_F(LinearMpcOutputTest, Output_Prediction_zero_state_Success) {
    StateSpace ss(A_, B_, C_, D_);
    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(mpc.nx());
    Eigen::VectorXd U(mpc.Nc() * mpc.nu());
    U.setConstant(1.0);

    Eigen::MatrixXd Y = mpc.prediction(x0, U);

    EXPECT_EQ(Y.rows(), mpc.Np() * mpc.ny());
    EXPECT_EQ(Y.cols(), 1);

    // Assuming C = [1 0], D = 0 → output = position from forced response
    Eigen::VectorXd expected_Y(mpc.Np() * mpc.ny());
    expected_Y <<
        0.000,
        0.010,
        0.027,
        0.049,
        0.073,
        0.110,
        0.134,
        0.148,
        0.156,
        0.158;

    EXPECT_MATRIX_NEAR(Y, expected_Y, tol, prec);
}


TEST_F(LinearMpcOutputTest, Output_Prediction_Success) {
    StateSpace ss(A_, B_, C_, D_);
    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    Eigen::VectorXd x0(mpc.nx());
    x0 << 1.0, 2.0;

    Eigen::VectorXd U(mpc.Nc() * mpc.nu());
    U << 1.0, 0.8, 0.4, -0.2, -0.6;

    Eigen::MatrixXd Y = mpc.prediction(x0, U);

    EXPECT_EQ(Y.size(), mpc.Np() * mpc.ny());

    // Assuming C = [1 0], D = 0 → y = position component
    Eigen::VectorXd expected_Y(mpc.Np() * mpc.ny());
    expected_Y <<
        1.200,
        1.330,
        1.405,
        1.435,
        1.426,
        1.379,
        1.317,
        1.246,
        1.171,
        1.093;

    EXPECT_MATRIX_NEAR(Y, expected_Y, tol, prec);
}


TEST_F(LinearMpcOutputTest, Output_Unc_Opt_Zero_State_Zero_Ref_Success) {
    StateSpace ss(A_, B_, C_, D_);
    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(mpc.nx());
    Eigen::VectorXd y_ref = Eigen::VectorXd::Zero(mpc.ny());

    Eigen::VectorXd expected_U_opt(mpc.Nc() * mpc.nu());
    expected_U_opt.setZero();

    EXPECT_CALL(*mock_qp_solver, solve(_, _, _, _, _, _))
        .WillOnce(Return(expected_U_opt));

    Eigen::VectorXd U = mpc.optimization(x0, y_ref);

    EXPECT_EQ(U.size(), mpc.Nc() * mpc.nu());
    EXPECT_TRUE(U.isZero(1e-10)) << "Optimal input should be zero when x0=0 and y_ref=0";
}


TEST_F(LinearMpcOutputTest, Output_Unc_Opt_Tracking_Constant_Ref_Success) {
    StateSpace ss(A_, B_, C_, D_);
    LinearMpcTestAccessor mpc(ss, mpc_data_, mock_qp_solver);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(mpc.nx());

    // Constant output reference
    Eigen::VectorXd y_ref(mpc.ny());
    y_ref.setZero();
    y_ref(0) = 1.0;

    Eigen::VectorXd Y_REF = y_ref.replicate(mpc.Np(), 1);

    // Correct stacked output mapping
    const Eigen::MatrixXd& Dp = mpc.Dp();

    // Free output prediction
    Eigen::VectorXd y_free = mpc.Cp() * x0;
    Eigen::VectorXd error = y_free - Y_REF;

    // Unconstrained QP
    Eigen::MatrixXd H = Dp.transpose() * mpc.Qp() * Dp + mpc.Rp();
    Eigen::VectorXd g = Dp.transpose() * mpc.Qp() * error;

    Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
    ASSERT_EQ(ldlt.info(), Eigen::Success)
        << "Hessian not positive definite";

    Eigen::VectorXd expected_U_opt = ldlt.solve(-g);

    EXPECT_CALL(*mock_qp_solver, solve(_, _, _, _, _, _))
        .WillOnce(Return(expected_U_opt));

    Eigen::VectorXd U = mpc.optimization(x0, y_ref);

    EXPECT_EQ(U.size(), mpc.Nc() * mpc.nu());
    EXPECT_TRUE(U.isApprox(expected_U_opt, 1e-8))
        << "Optimal input does not match analytical output-based solution";
}

