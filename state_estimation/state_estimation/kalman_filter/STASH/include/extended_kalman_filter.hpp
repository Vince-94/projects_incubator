#ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP

#include "system_models/system_models.hpp"

#include <Eigen/Dense>


class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter(SystemModels&& plant, SystemModels&& sensor);

    void initialize(const Eigen::VectorXd x0, const Eigen::MatrixXd P0);

    void initialize_at_first_update(const Eigen::VectorXd z, const Eigen::MatrixXd R);

    void predict(const Eigen::VectorXd u, const Eigen::MatrixXd Q);

    void update(const Eigen::VectorXd z, const Eigen::MatrixXd R);

    // Getters
    Eigen::VectorXd getStatePrio() { return _x_prio; };

    Eigen::MatrixXd getCovariancePrio() { return _P_prio; };

    Eigen::VectorXd getStatePost() { return _x_post; };

    Eigen::MatrixXd getCovariancePost() { return _P_post; };

    // Setters
    void setStatePrio(Eigen::VectorXd x_prio) { _x_prio = x_prio; };

    void setCovariancePrio(Eigen::MatrixXd P_prio) { _P_prio = P_prio; };

    void setStatePost(Eigen::VectorXd x_post) { _x_post = x_post; };

    void setCovariancePost(Eigen::MatrixXd P_post) { _P_post = P_post; };

private:
    /// @brief System model
    SystemModels _model;
    SystemModels _sensor;

    /// @brief System model dimension
    int _nx{};
    int _nu{};
    int _ny{};

    bool _initialized = false;

    // State/Covariance
    Eigen::VectorXd _x_prio{};
    Eigen::MatrixXd _P_prio{};
    Eigen::VectorXd _x_post{};
    Eigen::MatrixXd _P_post{};

};


#endif  // EXTENDED_KALMAN_FILTER_HPP