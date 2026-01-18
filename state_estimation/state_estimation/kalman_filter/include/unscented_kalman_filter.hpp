#ifndef UNSCENTED_KALMAN_FILTER_HPP
#define UNSCENTED_KALMAN_FILTER_HPP

#include "system_models.hpp"
#include "jacobians.hpp"

#include <Eigen/Dense>


class UnscentedKalmanFilter {
public:
    UnscentedKalmanFilter(NonlinearSystem&& system_model, double alpha, double kappa, double beta);

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

protected:
    void computeSigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P, Eigen::MatrixXd& sigma_points, Eigen::VectorXd& wm, Eigen::VectorXd& wc) const;

    Eigen::VectorXd recoverMean(const Eigen::MatrixXd& points, const Eigen::VectorXd& wm) const;

    Eigen::MatrixXd recoverCov(const Eigen::MatrixXd& points, const Eigen::VectorXd& mean, const Eigen::VectorXd& wc) const;

    Eigen::MatrixXd recoverCrossCov(const Eigen::MatrixXd& state_points, const Eigen::VectorXd& state_mean,
                                    const Eigen::MatrixXd& meas_points, const Eigen::VectorXd& meas_mean,
                                    const Eigen::VectorXd& wc) const;

private:
    /// @brief System model
    NonlinearSystem system_model_;

    /// @brief System model dimension
    int nx_{};
    int nu_{};
    int ny_{};

    bool initialized_ = false;

    double alpha_ = 1e-3;  // Spread (0 < alpha <=1)
    double kappa_ = 0.0;   // Often 3 - nx_
    double beta_ = 2.0;    // For Gaussian

    // State/Covariance
    Eigen::VectorXd x_prio_{};
    Eigen::MatrixXd P_prio_{};
    Eigen::VectorXd x_post_{};
    Eigen::MatrixXd P_post_{};

    Eigen::VectorXd u0_{};
};


#endif  // UNSCENTED_KALMAN_FILTER_HPP