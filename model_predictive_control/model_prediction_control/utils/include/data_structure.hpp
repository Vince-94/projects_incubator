#ifndef DATA_STRUCTURE_HPP
#define DATA_STRUCTURE_HPP

#include <Eigen/Geometry>


enum class SolverStatus {
    NotReady,
    Ok,
    Infeasible,
    Error
};


enum class OperatingPointEnum {
    EquilibriumPoint,
    CurrentState,
    PredictedTrajectory,
    _Count
};


struct StateSpace {
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;
    Eigen::MatrixXd D;
    int nx{0};
    int nu{0};
    int ny{0};

    StateSpace() = default;

    explicit StateSpace(
        Eigen::MatrixXd A_,
        Eigen::MatrixXd B_
    ) : A{std::move(A_)}, B{std::move(B_)} {
        nx = A.rows();
        nu = B.cols();

        if (A.cols() != nx) {
            throw std::invalid_argument("A must be square of dimension nx");  // TODO
        }

        if (B.rows() != nx) {
            throw std::invalid_argument("B.rows() must match A.rows()");  // TODO
        }
    }

    explicit StateSpace(
        Eigen::MatrixXd A_,
        Eigen::MatrixXd B_,
        Eigen::MatrixXd C_,
        Eigen::MatrixXd D_ = Eigen::MatrixXd()
    ) : StateSpace(std::move(A_), std::move(B_)) {
        C = std::move(C_);
        D = std::move(D_);
        ny = C.rows();

        if (C.cols() != nx) {
            std::string err_msg = "C column dimension (" + std::to_string(C.cols()) + ") must match A row dimension (" + std::to_string(A.rows()) + ")";
            throw std::invalid_argument(err_msg);
        }

        if (D.size() == 0) {
            D = Eigen::MatrixXd::Zero(ny, nu);
        } else {
            if (D.rows() != ny) {
                std::string err_msg = "D row dimension (" + std::to_string(D.cols()) + ") must match ny (" + std::to_string(ny) + ")";
                throw std::invalid_argument(err_msg);
            }
            if (D.cols() != nu) {
                std::string err_msg = "D column dimension (" + std::to_string(D.cols()) + ") must nu (" + std::to_string(nu) + ")";
                throw std::invalid_argument(err_msg);
            }
        }
    }
};


struct NonlinearSystem {
    using DynFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>;

    // Dimensions
    int nx{};
    int nu{};
    int ny{};

    DynFunc f{};
    DynFunc h{};

    NonlinearSystem(DynFunc f_, const Eigen::VectorXd& x0, const Eigen::VectorXd& u0) : f(std::move(f_)) {
        nx = x0.size();
        nu = u0.size();

        auto x_dot = f(x0, u0);

        if (nx != x_dot.size()) {
            throw std::runtime_error("f(x,u) must return vector of same size as x");
        }
    }

    NonlinearSystem(DynFunc f_, DynFunc h_,  const Eigen::VectorXd& x0, const Eigen::VectorXd& u0) : NonlinearSystem(f_, x0, u0) {
        auto y = h_(x0, u0);
        ny = y.size();
    }
};


struct OperativePoint {
    Eigen::VectorXd x;
    Eigen::VectorXd u;
};


struct MpcData {
    /// @brief Discretization method
    std::string discretization{};

    /// @brief State weight matrix
    Eigen::MatrixXd Q{};

    /// @brief Input weight matrix
    Eigen::MatrixXd R{};

    /// @brief Input rate weight matrix
    Eigen::MatrixXd S{};

    /// @brief Terminal state weight matrix
    Eigen::MatrixXd P{};

    /// @brief MPC Sample time [s]
    double Ts;

    /// @brief Prediction Horizon
    int Np{};

    /// @brief Control Horizon
    int Nc{};

    /// @brief State constraints
    Eigen::VectorXd x_min{}, x_max{};

    /// @brief Output constraints
    Eigen::VectorXd y_min{}, y_max{};

    /// @brief Input constraints
    Eigen::VectorXd u_min{}, u_max{};

    /// @brief Input rate constraints
    Eigen::VectorXd du_min{}, du_max{};

    OperatingPointEnum operating_point;

    Eigen::VectorXd x0{};

    Eigen::VectorXd u_eq{};
};



#endif  // DATA_STRUCTURE_HPP