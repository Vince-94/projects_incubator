#include "unscented_kalman_filter.hpp"

#include <iostream>


UnscentedKalmanFilter::UnscentedKalmanFilter(NonlinearSystem&& system_model, double alpha = 1e-3, double kappa = 0.0, double beta = 2.0) : system_model_{std::move(system_model)}, alpha_(alpha), kappa_(kappa), beta_(beta) {
    nx_ = system_model_.nx;
    nu_ = system_model_.nu;
    ny_ = system_model_.ny;
}


void UnscentedKalmanFilter::initialize(const Eigen::VectorXd x0, const Eigen::MatrixXd P0) {
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


void UnscentedKalmanFilter::initializeAtFirstUpdate(const Eigen::VectorXd z, const Eigen::MatrixXd R) {
    if (z.size() != ny_) {
        throw std::invalid_argument("z dimensions are inconsistent");
    }

    if (R.rows() != ny_ || R.cols() != ny_) {
        throw std::invalid_argument("R dimensions are inconsistent");
    }

    // Approximate nonlinear init: Linearize h around guess (e.g., zero), use pseudoinverse
    Eigen::VectorXd x_guess = Eigen::VectorXd::Zero(nx_);
    Eigen::VectorXd u_guess = Eigen::VectorXd::Zero(nu_);  // Or actual u if available
    auto Jhx = jacobian_dh_dx(system_model_.h, x_guess, u_guess);  // Jhx = dh/dx at guess
    auto pred_z = system_model_.h(x_guess, u_guess);
    auto y_init = z - pred_z + Jhx * x_guess;  // Linearized residual

    auto Ht_Rinv = Jhx.transpose() * R.inverse();
    auto info_matrix = Ht_Rinv * Jhx;

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


void UnscentedKalmanFilter::computeSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, Eigen::MatrixXd& sigma_points, Eigen::VectorXd& wm, Eigen::VectorXd& wc) const {
    double lambda = alpha_ * alpha_ * (nx_ + kappa_) - nx_;

    // Cholesky sqrt(P)
    Eigen::LLT<Eigen::MatrixXd> llt(P);
    if (llt.info() != Eigen::Success) throw std::runtime_error("P not positive definite");
    Eigen::MatrixXd sqrtP = llt.matrixL();  // Lower triangular

    sigma_points.resize(nx_, 2 * nx_ + 1);
    sigma_points.col(0) = x;
    double scale = std::sqrt(nx_ + lambda);
    for (int i = 0; i < nx_; ++i) {
        sigma_points.col(i + 1) = x + scale * sqrtP.col(i);
        sigma_points.col(i + nx_ + 1) = x - scale * sqrtP.col(i);
    }

    wm.resize(2 * nx_ + 1);
    wc.resize(2 * nx_ + 1);
    wm(0) = lambda / (nx_ + lambda);
    wc(0) = wm(0) + (1 - alpha_ * alpha_ + beta_);
    double w = 1.0 / (2 * (nx_ + lambda));
    wm.tail(2 * nx_).setConstant(w);
    wc.tail(2 * nx_).setConstant(w);
}


Eigen::VectorXd UnscentedKalmanFilter::recoverMean(const Eigen::MatrixXd& points, const Eigen::VectorXd& wm) const {
    return points * wm;
}


Eigen::MatrixXd UnscentedKalmanFilter::recoverCov(const Eigen::MatrixXd& points, const Eigen::VectorXd& mean, const Eigen::VectorXd& wc) const {
    int n = points.cols();
    Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(points.rows(), points.rows());
    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd diff = points.col(i) - mean;
        cov += wc(i) * diff * diff.transpose();
    }
    return cov;
}


Eigen::MatrixXd UnscentedKalmanFilter::recoverCrossCov(const Eigen::MatrixXd& state_points, const Eigen::VectorXd& state_mean,
                                                       const Eigen::MatrixXd& meas_points, const Eigen::VectorXd& meas_mean,
                                                       const Eigen::VectorXd& wc) const {
    int n = state_points.cols();
    Eigen::MatrixXd cross = Eigen::MatrixXd::Zero(state_points.rows(), meas_points.rows());
    for (int i = 0; i < n; ++i) {
        cross += wc(i) * (state_points.col(i) - state_mean) * (meas_points.col(i) - meas_mean).transpose();
    }
    return cross;
}


void UnscentedKalmanFilter::predict(const Eigen::VectorXd u, const Eigen::MatrixXd Q) {
    // Check if initialized
    if (!initialized_) return;

    // Check dimensions
    if (u.size() != nu_) throw std::invalid_argument("u dimensions inconsistent");
    if (Q.rows() != nx_ || Q.cols() != nx_) throw std::invalid_argument("Q dimensions inconsistent");

    // Generate sigma points around posterior
    Eigen::MatrixXd sigma_points;
    Eigen::VectorXd wm, wc;
    computeSigmaPoints(x_post_, P_post_, sigma_points, wm, wc);

    // Propagate through f
    Eigen::MatrixXd trans_points(nx_, 2 * nx_ + 1);
    for (int i = 0; i < 2 * nx_ + 1; ++i) {
        trans_points.col(i) = system_model_.f(sigma_points.col(i), u);
    }

    // Recover prior mean/cov
    x_prio_ = recoverMean(trans_points, wm);
    P_prio_ = recoverCov(trans_points, x_prio_, wc) + Q;
}


void UnscentedKalmanFilter::update(const Eigen::VectorXd z, const Eigen::MatrixXd R) {
    // Check dimensions
    if (z.size() != ny_) throw std::invalid_argument("z dimensions inconsistent");
    if (R.rows() != ny_ || R.cols() != ny_) throw std::invalid_argument("R dimensions inconsistent");

    // Check if initialized
    if (!initialized_) initializeAtFirstUpdate(z, R);

    // Generate sigma points around prior
    Eigen::MatrixXd sigma_points;
    Eigen::VectorXd wm, wc;
    computeSigmaPoints(x_prio_, P_prio_, sigma_points, wm, wc);

    // Propagate through h
    Eigen::MatrixXd trans_points(ny_, 2 * nx_ + 1);
    for (int i = 0; i < 2 * nx_ + 1; ++i) {
        trans_points.col(i) = system_model_.h(sigma_points.col(i), u0_);
    }

    // Recover predicted measurement mean/cov
    Eigen::VectorXd z_pred = recoverMean(trans_points, wm);
    Eigen::MatrixXd S = recoverCov(trans_points, z_pred, wc) + R;

    // Cross-covariance
    Eigen::MatrixXd P_xz = recoverCrossCov(sigma_points, x_prio_, trans_points, z_pred, wc);

    // Check S
    Eigen::LLT<Eigen::MatrixXd> llt(S);
    if (llt.info() != Eigen::Success) throw std::runtime_error("S not positive definite");

    // Gain and posterior
    auto K = P_xz * S.inverse();
    auto y = z - z_pred;
    x_post_ = x_prio_ + K * y;
    P_post_ = P_prio_ - K * S * K.transpose();  // Alternative form for stability
    P_post_ = 0.5 * (P_post_ + P_post_.transpose());  // Symmetrize
}
