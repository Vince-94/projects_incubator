#ifndef LINEAR_TIME_VARYING_MPC
#define LINEAR_TIME_VARYING_MPC

#include <array>
#include <vector>
#include <string>
#include <expected>
#include <memory>
#include <iostream>
#include <functional>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "data_structure.hpp"
#include "discretization.hpp"
#include "solver.hpp"



namespace mpc {


// Nonlinear dynamics signature
using Dynamics = std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>;


class LinearTimeVaryingMpc {
  public:
    LinearTimeVaryingMpc(const NonlinearSystem& system_model, const MpcData& mpc_data, std::shared_ptr<IQpSolver> qp_solver, bool debug);

    ~LinearTimeVaryingMpc() = default;

    LinearTimeVaryingMpc(const LinearTimeVaryingMpc&) = delete;

    LinearTimeVaryingMpc& operator=(const LinearTimeVaryingMpc&) = delete;

    LinearTimeVaryingMpc(const LinearTimeVaryingMpc&&) = delete;

    LinearTimeVaryingMpc& operator=(const LinearTimeVaryingMpc&&) = delete;

    Eigen::VectorXd onUpdate(const Eigen::VectorXd& x0, const Eigen::VectorXd& ref);

    // Getters
    StateSpace ssd() { return ssd_; };
    int nx() { return nx_; };
    int ny() { return ny_; };
    int nu() { return nu_; };

    double Ts() { return mpc_data_.Ts; };
    int Np() { return mpc_data_.Np; };
    int Nc() { return mpc_data_.Nc; };

    Eigen::MatrixXd P() { return mpc_data_.P; };
    Eigen::MatrixXd Q() { return mpc_data_.Q; };
    Eigen::MatrixXd R() { return mpc_data_.R; };
    Eigen::MatrixXd S() { return mpc_data_.S; };
    Eigen::VectorXd x_min() { return mpc_data_.x_min; };
    Eigen::VectorXd x_max() { return mpc_data_.x_max; };
    Eigen::VectorXd y_min() { return mpc_data_.y_min; };
    Eigen::VectorXd y_max() { return mpc_data_.y_max; };
    Eigen::VectorXd u_min() { return mpc_data_.u_min; };
    Eigen::VectorXd u_max() { return mpc_data_.u_max; };
    Eigen::VectorXd du_min() { return mpc_data_.du_min; };
    Eigen::VectorXd du_max() { return mpc_data_.du_max; };

    Eigen::MatrixXd Ap() { return Ap_; };
    Eigen::MatrixXd Bp() { return Bp_; };
    Eigen::MatrixXd Cp() { return Cp_; };
    Eigen::MatrixXd Dp() { return Dp_; };

    Eigen::MatrixXd Qp() { return Qp_; };
    Eigen::MatrixXd Rp() { return Rp_; };
    Eigen::MatrixXd Sp() { return Sp_; };
    Eigen::VectorXd X_min() { return X_min_; };
    Eigen::VectorXd X_max() { return X_max_; };
    Eigen::VectorXd Y_min() { return Y_min_; };
    Eigen::VectorXd Y_max() { return Y_max_; };
    Eigen::VectorXd U_min() { return U_min_; };
    Eigen::VectorXd U_max() { return U_max_; };
    Eigen::VectorXd dU_min() { return dU_min_; };
    Eigen::VectorXd dU_max() { return dU_max_; };

  protected:
    Eigen::VectorXd computeEquilibriumPoint(
        const NonlinearSystem& nonlin_system,
        const Eigen::VectorXd& x0,
        const Eigen::VectorXd& u_eq,
        double tol = 1e-8,
        int max_iter = 50
    );

    void checkDimensions();

    void precomputeMatrices();

    void initSolver(bool debug);

    void getOperatingPoint();

    Eigen::VectorXd rk4(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double Ts);

    StateSpace linearization(const NonlinearSystem& f, const Eigen::VectorXd& x0, const Eigen::VectorXd& u0);

    Eigen::MatrixXd prediction(const Eigen::VectorXd& x0, const Eigen::VectorXd& U);

    void computePredictionMatrices();

    Eigen::VectorXd optimization(const Eigen::VectorXd& x0, const Eigen::VectorXd& x_ref);

  private:
    /*
     * #########################
     * #   System parameters   #
     * #########################
     */

    NonlinearSystem system_model_;

    /// @brief Discretized System State Space
    StateSpace ssd_;

    /// @brief State dimension
    int nx_{};

    /// @brief Control input dimension
    int nu_{};

    /// @brief Output dimension
    int ny_{};

    /// @brief Initial state vector of dimension nx
    Eigen::VectorXd x0_{};

    /// @brief State vector of dimension nx
    Eigen::VectorXd x_{};

    /// @brief Output vector of dimension ny
    Eigen::VectorXd y_{};

    /// @brief Reference vector
    Eigen::VectorXd ref_{};

    /// @brief Optimal input
    Eigen::VectorXd u_opt_{};

    /// @brief Initial state equilibrium point
    Eigen::VectorXd x_eq_{};

    /// @brief Initial input equilibrium point
    Eigen::VectorXd u_eq_{};

    OperativePoint op_point_;

    /// @brief QP solver
    std::shared_ptr<IQpSolver> qp_solver_{};

    /*
     * ######################
     * #   MPC parameters   #
     * ######################
     */

    // MPC data
    MpcData mpc_data_{};

    /*
     * ###########################
     * #   Prediction matrices   #
     * ###########################
     */

    Eigen::MatrixXd Ap_{};
    Eigen::MatrixXd Bp_{};
    Eigen::MatrixXd Cp_{};
    Eigen::MatrixXd Dp_{};
    Eigen::VectorXd U_{};
    Eigen::VectorXd U_stacked_{};  // U_stacked_ = [u_{k-1}; 0; 0; ...]
    Eigen::MatrixXd M_{};
    Eigen::MatrixXd Qp_{};
    Eigen::MatrixXd Rp_{};
    Eigen::MatrixXd Sp_{};
    Eigen::VectorXd X_min_{};
    Eigen::VectorXd X_max_{};
    Eigen::VectorXd Y_min_{};
    Eigen::VectorXd Y_max_{};
    Eigen::VectorXd U_min_{};
    Eigen::VectorXd U_max_{};
    Eigen::VectorXd dU_min_{};
    Eigen::VectorXd dU_max_{};

    std::vector<Eigen::MatrixXd> A_pow_{};
    std::vector<Eigen::MatrixXd> CA_pow_{};


    /*
     * #################
     * #   QP Solver   #
     * #################
     */

    // proxsuite::proxqp::dense::QP<double> qp;

};

}


#endif  // LINEAR_TIME_VARYING_MPC