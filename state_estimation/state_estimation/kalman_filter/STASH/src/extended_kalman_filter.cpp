#include "extended_kalman_filter.hpp"

#include <iostream>


ExtendedKalmanFilter::ExtendedKalmanFilter(SystemModels&& model) : _model{std::move(model)}, _sensor{std::move(sensor)}
{
    _nx = _model.get_nx();
    _nu = _model.get_nu();
    _ny = _model.get_ny();
    _C = _model.get_C();
}


void ExtendedKalmanFilter::initialize(const Eigen::VectorXd x0, const Eigen::MatrixXd P0)
{
    // Check if already initialized
    if (_initialized) {
        return;
    }

    // Check dimensions consistency
    if (x0.size() != _nx) {
        throw std::invalid_argument("x0 dimensions are not consistant");
    }

    if (P0.rows() != _nx || P0.cols() != _nx) {
        throw std::invalid_argument("P0 dimensions are not consistant");
    }

    // A priori state/covariance initialization
    _x_prio.resize(_nx);
    _x_prio.setZero();

    _P_prio.resize(_nx, _nx);
    _P_prio.setZero();

    // A posteriori state/covariance initialization
    _x_post.resize(_nx);
    _x_post = x0;

    _P_post.resize(_nx, _nx);
    _P_post = P0;

    _initialized = true;
}


void ExtendedKalmanFilter::initialize_at_first_update(const Eigen::VectorXd z, const Eigen::MatrixXd R)
{
    if (z.size() != _ny) {
        throw std::invalid_argument("z dimensions are not consistant");
    }

    if (R.rows() != _ny || R.cols() != _ny) {
        throw std::invalid_argument("R dimensions are not consistant");
    }

    // A priori state/covariance initialization
    _x_prio.resize(_nx);
    _x_prio = _C.transpose()*z;

    _P_prio.resize(_nx, _nx);
    _P_prio = _C.transpose()*R*_C;

    // A posteriori state/covariance initialization
    _x_post.resize(_nx);
    _x_post.setZero();

    _P_post.resize(_nx, _nx);
    _P_post.setZero();

    _initialized = true;
}


void ExtendedKalmanFilter::predict(const Eigen::VectorXd u, const Eigen::MatrixXd Q)
{
    // Check if initialized
    if (!_initialized) return;

    auto _x_prio = _model.update_state(u);
    auto _P_prio = _model.update_covariance(Q);
}


void ExtendedKalmanFilter::update(const Eigen::VectorXd z, const Eigen::MatrixXd R)
{
    // Check if initialized
    if (!_initialized) initialize_at_first_update(z, R);

    // TODO

    // Measurement error: y(k) = z(k) - h(x, 0)
    auto z = _sensor->update_measurement(_x_prio);

    // Covariance error: Jhx Pk Jhx^T + Jhv Rk Jhv^T
    Jhx = _sensor->get_Jhx();
    auto S = Jhx * _P_prio * Jhx.transpose() + _sensor->getCovariance();

    // Kalman gain: K(k) = P_prio Jhx^T S^(-1)
    auto K = _P_prio * Jhx.transpose() * S.inverse();

    // State posteriori: x_post(k) = x_prio(k) + K(k) v(k)
    _x_post = _x_prio + K*z;

    // Convariance posteriori: P_post(K) = (I - K(k) Jhx) P_prio(k)
    _P_post = (Eigen::MatrixXd::Identity(_nx, _nx) - K * Jhx.transpose()) * _P_prio;
}
