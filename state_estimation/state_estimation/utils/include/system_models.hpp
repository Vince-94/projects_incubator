#ifndef DATA_STRUCTURE_HPP
#define DATA_STRUCTURE_HPP

#include <Eigen/Geometry>



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

    Eigen::VectorXd updateState(Eigen::VectorXd x0, Eigen::VectorXd u) {
        // Dimension validation
        if (x0.size() != nx || u.size() != nu) {
            std::string error_message = "u has unexpected dimensions. Expected: (" + std::to_string(nu) + "). Actual: (" + std::to_string(u.size()) + ")";
            throw std::invalid_argument(error_message);
        }

        // x(k+1) = A x(k) + B u(k)
        return A * x0 + B * u;
    }

    Eigen::MatrixXd updateCovariance(Eigen::MatrixXd P0, Eigen::MatrixXd Q) {
        // Dimension validation
        if (P0.rows() != nx || P0.cols() != nx) {
            throw std::invalid_argument(
                "P0 must be square and have size " + std::to_string(nx) + "×" + std::to_string(nx) +
                " (got " + std::to_string(P0.rows()) + "×" + std::to_string(P0.cols()) + ")"
            );
        }

        if (Q.rows() != nu || Q.cols() != nu) {
            throw std::invalid_argument(
                "Q must be square and have size " + std::to_string(nu) + "×" + std::to_string(nu) +
                " (got " + std::to_string(Q.rows()) + "×" + std::to_string(Q.cols()) + ")"
            );
        }

        // P(k+1) = A P A^T + B Q B^T
        return A * P0 * A.transpose() + B * Q * B.transpose();
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



#endif  // DATA_STRUCTURE_HPP