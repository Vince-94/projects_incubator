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
};



#endif  // DATA_STRUCTURE_HPP