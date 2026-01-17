#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>

#include "system_models.hpp"


class KalmanFilter
{
public:
    virtual void initialize(const Eigen::VectorXd x0, const Eigen::MatrixXd P0) = 0;

    virtual void initialize_at_first_update(const Eigen::VectorXd z, const Eigen::MatrixXd R) = 0;

    virtual void predict(const Eigen::VectorXd u, const Eigen::MatrixXd Q) = 0;

    virtual void update(const Eigen::VectorXd z, const Eigen::MatrixXd R) = 0;

    // Getters
    Eigen::VectorXd get_x_prio() { return _x_prio; };

    Eigen::MatrixXd get_P_prio() { return _P_prio; };

    Eigen::VectorXd get_x_post() { return _x_post; };

    Eigen::MatrixXd get_P_post() { return _P_post; };

    // Setters
    void set_x_prio(Eigen::VectorXd x_prio) { _x_prio = x_prio; };

    void set_P_prio(Eigen::MatrixXd P_prio) { _P_prio = P_prio; };

    void set_x_post(Eigen::VectorXd x_post) { _x_post = x_post; };

    void set_P_post(Eigen::MatrixXd P_post) { _P_post = P_post; };

protected:
    /// @brief System model
    SystemModels _model;

    /// @brief Sensor model
    Eigen::MatrixXd _C{};  //TODO

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


#endif  // KALMAN_FILTER_HPP