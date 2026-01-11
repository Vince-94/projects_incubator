#ifndef DISCRETIZATION_HPP
#define DISCRETIZATION_HPP

#include "data_structure.hpp"



inline StateSpace systemDiscretization(const StateSpace& ss, const double Ts, const std::string mode) {
    if (mode == "Euler") {
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ss.A.rows(), ss.A.rows());

        Eigen::MatrixXd Ad = I + ss.A * Ts;
        Eigen::MatrixXd Bd = ss.B * Ts;

        if (ss.ny == 0) {
            return StateSpace{Ad, Bd};
        }
        return StateSpace{Ad, Bd, ss.C, ss.D};

    } else if (mode == "ZOH") {
        int n = ss.A.rows();

        // Build the augmented matrix for exact ZOH discretization
        Eigen::MatrixXd M(2 * n, 2 * n);
        M << ss.A, ss.B,
            Eigen::MatrixXd::Zero(ss.nu, n), Eigen::MatrixXd::Zero(ss.nu, ss.nu);

        // Compute matrix exponential
        Eigen::MatrixXd expM = (M * Ts).exp();

        // Extract Ad and Bd
        Eigen::MatrixXd Ad = expM.topLeftCorner(n, n);
        Eigen::MatrixXd Bd = expM.topRightCorner(n, ss.nu);

        if (ss.ny == 0) {
            return StateSpace{Ad, Bd};
        }
        return StateSpace{Ad, Bd, ss.C, ss.D};

    } else {
        std::string err_msg = "Error: No discretization mode matched: " + mode;
        throw std::invalid_argument(err_msg);
    }
}


#endif  // DISCRETIZATION_HPP