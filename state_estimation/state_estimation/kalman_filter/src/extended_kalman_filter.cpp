#include "extended_kalman_filter.hpp"

#include <iostream>


ExtendedKalmanFilter::ExtendedKalmanFilter(NonlinearSystem&& system_model) : system_model_{std::move(system_model)} {
    nx_ = system_model_.nx;
    nu_ = system_model_.nu;
    ny_ = system_model_.ny;
}


void ExtendedKalmanFilter::initialize(const Eigen::VectorXd x0, const Eigen::MatrixXd P0) {
    // Check if already initialized
    if (initialized_) {
        std::cout << "EKF already initialized" << std::endl;
        return;
    }

    std::cout << "EKF initializing..." << std::endl;

    // Check dimensions consistency
    if (x0.size() != nx_) {
        throw std::invalid_argument("x0 dimensions are inconsistent");
    }

    if (P0.rows() != nx_ || P0.cols() != nx_) {
        throw std::invalid_argument("P0 dimensions are inconsistent");
    }

    // A priori state/covariance initialization
    x_prio_.setZero(nx_);
    P_prio_.setZero(nx_, nx_);

    // A posteriori state/covariance initialization
    x_post_.setZero(nx_);
    x_post_ = x0;

    P_post_.setZero(nx_, nx_);
    P_post_ = P0;

    // Zero u vector
    u0_ = Eigen::VectorXd::Zero(nu_);

    initialized_ = true;
}


void ExtendedKalmanFilter::initializeAtFirstUpdate(const Eigen::VectorXd z, const Eigen::MatrixXd R) {
    if (z.size() != ny_) {
        throw std::invalid_argument("z dimensions are inconsistent");
    }

    if (R.rows() != ny_ || R.cols() != ny_) {
        throw std::invalid_argument("R dimensions are inconsistent");
    }

    // Approximate nonlinear init: Linearize h around guess (e.g., zero), use pseudoinverse
    Eigen::VectorXd x_guess = Eigen::VectorXd::Zero(nx_);
    Eigen::VectorXd u_guess = Eigen::VectorXd::Zero(nu_);  // Or actual u if available
    auto H = jacobian_dh_dx(system_model_.h, x_guess, u_guess);  // dh/dx at guess
    auto pred_z = system_model_.h(x_guess, u_guess);
    auto y_init = z - pred_z + H * x_guess;  // Linearized residual

    auto Ht_Rinv = H.transpose() * R.inverse();
    auto info_matrix = Ht_Rinv * H;

    // Use SelfAdjointEigenSolver for singularity
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(info_matrix);
    if (solver.info() != Eigen::Success) throw std::runtime_error("Eigen decomposition failed");
    auto eigenvalues = solver.eigenvalues();
    auto eigenvectors = solver.eigenvectors();
    const double tol = 1e-10;
    const double large_var = 1e9;
    Eigen::VectorXd pseudo_diag(nx_);
    for (Eigen::Index i = 0; i < nx_; ++i) {
        pseudo_diag(i) = (eigenvalues(i) > tol) ? 1.0 / eigenvalues(i) : large_var;
    }
    auto P_prio_init = eigenvectors * pseudo_diag.asDiagonal() * eigenvectors.transpose();

    x_prio_ = P_prio_init * (Ht_Rinv * y_init);
    P_prio_ = P_prio_init;

    x_post_.setZero(nx_);
    P_post_.setZero(nx_, nx_);
    initialized_ = true;
}


void ExtendedKalmanFilter::predict(const Eigen::VectorXd u, const Eigen::MatrixXd Q) {
    // Check if initialized
    if (!initialized_) return;

    // Check dimensions
    if (u.size() != nu_) throw std::invalid_argument("u dimensions inconsistent");
    if (Q.rows() != nx_ || Q.cols() != nx_) throw std::invalid_argument("Q dimensions inconsistent");

    // State a-priori (nonlinear) prediciton
    x_prio_ = system_model_.f(x_post_, u);

    // Covariance a-priori prediction
    auto Jfx = jacobian_df_dx(system_model_.f, x_post_, u);  // Jacobian: Jfx = df/dx
    auto Jfu = jacobian_df_du(system_model_.f, x_post_, u);  // Jacobian: Jfu = df/du  // TODO useful only if hasAdditiveProcessNoise == false
    P_prio_ = system_model_.updateCovariance(P_post_, Q, Jfx, Jfu);
}


void ExtendedKalmanFilter::update(const Eigen::VectorXd z, const Eigen::MatrixXd R) {
    // Check dimensions
    if (z.size() != ny_) throw std::invalid_argument("z dimensions inconsistent");
    if (R.rows() != ny_ || R.cols() != ny_) throw std::invalid_argument("R dimensions inconsistent");

    // Check if initialized
    if (!initialized_) initializeAtFirstUpdate(z, R);

    // Nonlinear innovation: y(k) = z(k) - h(x, 0)
    auto y = z - system_model_.h(x_prio_, u0_);

    // Innovation covariance: Jhx Pk Jhx^T + Jhv Rk Jhv^T
    auto H = jacobian_dh_dx(system_model_.h, x_prio_, u0_);  // Jacobian for measurement: Jhx = dh/dx
    auto S = H * P_prio_ * H.transpose() + R;

    // Check S invertibility via LLT
    Eigen::LLT<Eigen::MatrixXd> llt(S);
    if (llt.info() != Eigen::Success) throw std::runtime_error("S not positive definite");

    // Kalman gain: K(k) = P_prio Jhx^T S^(-1)
    auto K = P_prio_ * H.transpose() * S.inverse();

    // State posteriori: x_post(k) = x_prio(k) + K(k) v(k)
    x_post_ = x_prio_ + K * y;

    // Convariance posteriori: P_post(K) = (I - K(k) Jhx) P_prio(k)
    P_post_ = (Eigen::MatrixXd::Identity(nx_, nx_) - K * H) * P_prio_;

    // Enforce symmetry: P_post_ = 1/2 * (P_post + P_post^T);
    P_post_ = 0.5 * (P_post_ + P_post_.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P_post_);
    if (es.info() == Eigen::Success) {
        Eigen::VectorXd ev = es.eigenvalues();
        for (auto& v : ev) if (v < 0) v = 0;
        P_post_ = es.eigenvectors() * ev.asDiagonal() * es.eigenvectors().transpose();
    }
}
