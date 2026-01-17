#include "unscent_kalman_filter.hpp"

#include <cmath>
#include <iostream>
#include <vector>


ExtendedKalmanFilter::ExtendedKalmanFilter(SystemModels* model, SensorModels* sensor) : _model{model}, _sensor{sensor} {}


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
    // A priori state/covariance initialization
    _x_prio.resize(_nx);
    _x_prio.segment(0, _nu) = z;

    _P_prio.resize(_nx, _nx);
    _P_prio.block(0, 0, _nu, _nu) = R.diagonal().asDiagonal();
    _P_prio.block(0, _nu, _nu, _nu).setZero();
    _P_prio.block(_nu, 0, _nu, _nu).setZero();
    _P_prio.block(_nu, _nu, _nu, _nu).setZero();

    // A posteriori state/covariance initialization
    _x_post.resize(_nx);
    _x_post.setZero();

    _P_post.resize(_nx, _nx);
    _P_post.setZero();

    _initialized = true;
}


void ExtendedKalmanFilter::predict(const Eigen::VectorXd u, const Eigen::MatrixXd Q, const double t)
{
    // Check if initialized
    if (!_initialized) return;


    // Calculate sigma points
    std::vector s_points{};  //TODO vector
    std::vector k = {0, 1, 2, 3, 4};
    int nx = 5;

    for (int i = 0; i < nx; i++) {
        dx = sqrt((nx + k[i]) * _P_post)
        auto xi_p = _x_post + dx;
        auto xi_n = _x_post - dx;

        s_points.push_back(xi_p);
        s_points.push_back(xi_n);
    }

    // Calculate weights
    std::vector W = {k / (nx + k)};
    for (int i = 0; i < nx; i++) {
        auto Wi = k / (2*(nx + k))
        W.push_back(Wi);
        W.push_back(-Wi);
    }


    // Calculate prediction
    double _x_prio = 0;  //TODO vector
    double _P_prio = 0;  //TODO matrix
    for (int i = 0; i <= 2*nx && i < s_points.size(); ++i) {
        // state
        auto x_pred = _model->on_step(s_points[i], _P_post, u, Q, t);
        _x_prio += W[i] * x_pred;

        // covariance
        _P_prio += W[i] * (s_points[i] - _x_prio) * (s_points[i] - _x_prio).transpose() + Q;
    }

}


void ExtendedKalmanFilter::update(const Eigen::VectorXd z, const Eigen::MatrixXd R)
{
    // Check if initialized
    if (!_initialized) initialize_at_first_update(z, R);

    double z_pred = 0;  //TODO vector
    for (int i = 0; i <= 2*nx && i < s_points.size(); ++i) {
        // Measurement error: y(k) = ...
        _sensor->on_step(_x_prio);
        z_pred += W[i] * _sensor->getMeasurement();
    }

    // Measurement innovation
    auto v = z - z_pred;

    // Measurement covariance innovation
    double S = 0;  //TODO matrix
    for (int i = 0; i <= 2*nx && i < s_points.size(); ++i) {
        S += W[i] * (_sensor->getMeasurement() - z_pred) * (_sensor->getMeasurement() - z_pred).transpose() + R;
    }

    // Cross covariance
    double Pxz = 0;  //TODO matrix
    for (int i = 0; i <= 2*nx && i < s_points.size(); ++i) {
        Pxz += W[i] * (s_points[i] - _x_prio) * (_sensor->getMeasurement() - z_pred).transpose() + R;
    }

    // Kalman gain: K(k) = P_prio Jhx^T S^(-1)
    auto K = Pxz * S.inverse();

    // State posteriori: x_post(k) = x_prio(k) + K(k) v(k)
    _x_post = _x_prio + K*v;

    // Convariance posteriori: P_post(K) = P_prio(k) - K(k) S(k) K(k)^T
    _P_post = _P_prio(k) - K * S * K.transpose();

}
