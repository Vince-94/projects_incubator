#ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP

#include "system_models.hpp"
#include "jacobians.hpp"

#include <Eigen/Dense>


class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(NonlinearSystem&& system_model);

    void initialize(const Eigen::VectorXd x0, const Eigen::MatrixXd P0);

    void initializeAtFirstUpdate(const Eigen::VectorXd z, const Eigen::MatrixXd R);

    void predict(const Eigen::VectorXd u, const Eigen::MatrixXd Q);

    void update(const Eigen::VectorXd z, const Eigen::MatrixXd R);

    // Getters
    Eigen::VectorXd getStatePrio() { return x_prio_; };

    Eigen::MatrixXd getCovariancePrio() { return P_prio_; };

    Eigen::VectorXd getStatePost() { return x_post_; };

    Eigen::MatrixXd getCovariancePost() { return P_post_; };

    // Setters
    void setStatePrio(Eigen::VectorXd x_prio) { x_prio_ = x_prio; };

    void setCovariancePrio(Eigen::MatrixXd P_prio) { P_prio_ = P_prio; };

    void setStatePost(Eigen::VectorXd x_post) { x_post_ = x_post; };

    void setCovariancePost(Eigen::MatrixXd P_post) { P_post_ = P_post; };

private:
    /// @brief System model
    NonlinearSystem system_model_;

    /// @brief System model dimension
    int nx_{};
    int nu_{};
    int ny_{};

    bool initialized_ = false;

    // State/Covariance
    Eigen::VectorXd x_prio_{};
    Eigen::MatrixXd P_prio_{};
    Eigen::VectorXd x_post_{};
    Eigen::MatrixXd P_post_{};

    Eigen::VectorXd u0_{};
};


#endif  // EXTENDED_KALMAN_FILTER_HPP