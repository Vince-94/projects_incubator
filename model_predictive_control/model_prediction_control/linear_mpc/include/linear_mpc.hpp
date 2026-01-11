#ifndef LINEAR_MPC
#define LINEAR_MPC

#include <array>
#include <vector>
#include <string>
#include <expected>
#include <memory>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "data_structure.hpp"
#include "discretization.hpp"
#include "solver.hpp"



namespace mpc {


class LinearMpc {
  public:
    LinearMpc(const StateSpace& ss, const MpcData& mpc_data, std::shared_ptr<IQpSolver> qp_solver, bool debug = false);

    ~LinearMpc() = default;

    LinearMpc(const LinearMpc&) = delete;

    LinearMpc& operator=(const LinearMpc&) = delete;

    LinearMpc(const LinearMpc&&) = delete;

    LinearMpc& operator=(const LinearMpc&&) = delete;

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

    // Setters  // TODO unused
    // void Np(cont int Np) { Np_ = Np; };
    // void Nc(cont int Nc) { Nc_ = Nc_; };

    // void Q(const Eigen::MatrixXd Q) { Q_ = Q; };
    // void R(const Eigen::MatrixXd R) { R_ = R; };
    // void x_min(const Eigen::VectorXd x_min) { x_min_ = x_min; };
    // void x_max(const Eigen::VectorXd x_max) { x_max_ = x_max; };
    // void u_min(const Eigen::VectorXd u_min) { u_min_ = u_min; };
    // void u_max(const Eigen::VectorXd u_max) { u_max_ = u_max; };
    // void du_min(const Eigen::VectorXd du_min) { du_min_ = du_min; };
    // void du_max(const Eigen::VectorXd du_max) { du_max_ = du_max; };

  protected:
    void checkDimensions();

    void precomputeMatrices();

    void precomputePrediction();

    void initSolver(bool debug);

    Eigen::MatrixXd prediction(const Eigen::VectorXd& x0, const Eigen::VectorXd& U);

    Eigen::VectorXd optimization(const Eigen::VectorXd& x0, const Eigen::VectorXd& x_ref);

  private:
    /*
     * #########################
     * #   System parameters   #
     * #########################
     */

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

    /// @brief QP solver
    std::shared_ptr<IQpSolver> qp_solver_{};

    /*
     * ######################
     * #   MPC parameters   #
     * ######################
     */

    // /// @brief MPC Sample time [s]
    // double Ts_;

    // // Prediction Horizon
    // int Np_{};

    // // Control Horizon
    // int Nc_{};

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

    /*
     * #################
     * #   QP Solver   #
     * #################
     */

    // proxsuite::proxqp::dense::QP<double> qp;

};

}


#endif  // LINEAR_MPC