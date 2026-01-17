#include "linear_kalman_filter.hpp"

#include <iostream>


LinearKalmanFilter::LinearKalmanFilter(StateSpace&& system_model) : system_model_{std::move(system_model)} {
    nx_ = system_model_.nx;
    nu_ = system_model_.nu;
    ny_ = system_model_.ny;
    C_ = system_model_.C;
}


void LinearKalmanFilter::initialize(const Eigen::VectorXd x0, const Eigen::MatrixXd P0) {
    // Check if already initialized
    if (initialized_) {
        std::cout << "LKF already initialized" << std::endl;
        return;
    }

    std::cout << "LKF initializing..." << std::endl;

    // Check dimensions consistency
    if (x0.size() != nx_) {
        throw std::invalid_argument("x0 dimensions are not consistant");
    }

    if (P0.rows() != nx_ || P0.cols() != nx_) {
        throw std::invalid_argument("P0 dimensions are not consistant");
    }

    // A priori state/covariance initialization
    x_prio_.setZero(nx_);
    P_prio_.setZero(nx_, nx_);

    // A posteriori state/covariance initialization
    x_post_.setZero(nx_);
    x_post_ = x0;

    P_post_.setZero(nx_, nx_);
    P_post_ = P0;

    initialized_ = true;
}


void LinearKalmanFilter::initializeAtFirstUpdate(const Eigen::VectorXd z, const Eigen::MatrixXd R) {
    if (z.size() != ny_) {
        throw std::invalid_argument("z dimensions are not consistant");
    }

    if (R.rows() != ny_ || R.cols() != ny_) {
        throw std::invalid_argument("R dimensions are not consistant");
    }

    // A priori state/covariance initialization
    auto Ct_Rinv = C_.transpose() * R.inverse();
    auto info_matrix = Ct_Rinv * C_;  // Information matrix: Lambda = C^T R^{-1} C  // TODO preferring using SelfAdjointEigenSolver

    auto P_prio_init = info_matrix.inverse();

    x_prio_ = P_prio_init * (Ct_Rinv * z);
    P_prio_ = P_prio_init;

    // A posteriori state/covariance initialization
    x_post_.setZero(nx_);
    P_post_.setZero(nx_, nx_);

    initialized_ = true;
}


void LinearKalmanFilter::predict(const Eigen::VectorXd u, const Eigen::MatrixXd Q) {
    // Check if initialized
    if (!initialized_) return;

    // Check dimensions
    if (u.size() != nu_) throw std::invalid_argument("u dimensions inconsistent");
    if (Q.rows() != nx_ || Q.cols() != nx_) throw std::invalid_argument("Q dimensions inconsistent");

    // State a-priori prediciton
    x_prio_ = system_model_.updateState(x_post_, u);

    // Covariance a-priori prediction
    P_prio_ = system_model_.updateCovariance(P_post_, Q);
}


void LinearKalmanFilter::update(const Eigen::VectorXd z, const Eigen::MatrixXd R) {
    // Check dimensions
    if (z.size() != ny_) throw std::invalid_argument("z dimensions inconsistent");
    if (R.rows() != ny_ || R.cols() != ny_) throw std::invalid_argument("R dimensions inconsistent");

    // Check if initialized
    if (!initialized_) initializeAtFirstUpdate(z, R);

    // Measurement error: y(k) = z(k) - C(k) * x_prio(k)
    auto y = z - C_ * x_prio_;

    // Measurement error covariance: S(k) = C(k) P_prio(k)^T + M(k) R(k) M(k)^T
    auto S = C_ * P_prio_ * C_.transpose() + R;

    // Kalman gain: K(k) = P_prio C^T S^(-1)
    auto K = P_prio_ * C_.transpose() * S.inverse();

    // State posteriori: x_post(k) = x_prio(k) + K(k) y(k)
    x_post_ = x_prio_ + K * y;

    // Convariance posteriori: P_post(K) = (I - K(k) C(k)) P_prio(k)
    P_post_ = (Eigen::MatrixXd::Identity(nx_, nx_) - K * C_) * P_prio_;
}
