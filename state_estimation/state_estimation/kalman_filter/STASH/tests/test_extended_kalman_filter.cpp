#include "extended_kalman_filter.hpp"
#include "system_models/system_models.hpp"

#include <Eigen/Dense>

#include <gtest/gtest.h>


// https://github.com/Vince-94/kalman_filter/commit/51748a7edce6e5c242f7e949b6a3f7bd7df7e1dc#diff-e818340efa78a4cbe7442c1810a89a0bda2b96f099c3bf345015f7f26acf822b


//! Fixture
class FixtureEKF : public ::testing::Test {
protected:
    void SetUp() override {
        px = 0;
        py = 0;
        v = 5;
        psi = 0;

        init_pos_std = 2;
        init_vel_std = 10;
        init_heading_std = 1;
        acc_std = 0.1;
        gps_pos_std = 3.0;

        x0 << px, py, v, psi;
        P0.diagonal() << init_pos_std, init_pos_std, init_vel_std, init_heading_std;
        C << 1, 0, 0, 0,
             0, 1, 0, 0;
        Q.diagonal() << acc_std*acc_std;
        R.diagonal() << gps_pos_std*gps_pos_std, gps_pos_std*gps_pos_std;
    }

    double Ts = 0.1;
    const int nx = 4;
    const int nu = 2;
    const int ny = 2;

    Eigen::VectorXd x0{nx};      // initial state
    Eigen::MatrixXd P0{nx, nx};  // initial covariance matrix
    Eigen::MatrixXd C{ny, nx};   // output matrix
    Eigen::VectorXd u{nu};       // input vector
    Eigen::VectorXd z{ny};       // measurement vector
    Eigen::MatrixXd Q{nu, nu};   // incertainty matrix on input
    Eigen::MatrixXd R{ny, ny};   // incertainty matrix on sensor measurements
    Eigen::MatrixXd M{ny, ny};   //TODO unused [Eigen::MatrixXd::Identity{ny, ny};]

    // State
    double p{};     // [m]
    double v{};     // [m/s]

    // Input
    double a{};         // [m/s^2]
    double psi_dot{};   // [rad/s]

    // Variance
    double init_pos_std{};      // initial position standard deviation
    double init_vel_std{};      // initial velocity standard deviation
    double init_heading_std{};  // initial heading standard deviation
    double acc_std{};           // accelleration standard deviation
    double gps_pos_std{};       // gps position standard deviation
};


TEST_F(FixtureLKF, initialize) {

    // System model
    std::string palnt_model_name = "point_kinematic_2d_model";
    SystemModels model(palnt_model_name, Ts, x0, P0, C);

    // Sensor model
    std::string sensor_model_name = "point_kinematic_2d_model";


    ExtendedKalmanFilter ekf(std::move(model));


    // x_prio_expect << z(0),   z(1),   0, 0;
    // P_prio_expect << R(0,0), 0,      0, 0,
    //                  0,      R(1,1), 0, 0,
    //                  0,      0,      0, 0,
    //                  0,      0,      0, 0;

}



TEST_F(FixtureLKF, prediction_init) {

    // x_prio_expect << 0, 0, 5,  0;
    // P_prio_expect << 2, 0, 0,  0,
    //                  0, 2, 0,  0,
    //                  0, 0, 10, 0,
    //                  0, 0, 0,  1;



    // x_prio_expect << 0.51, 0.001, 5.2,  0.02;
    // P_prio_expect << 2.1,  0,     1.0,  0,
    //                  0,    2.01,  0,    0.1,
    //                  1.0,  0,     10.0, 0,
    //                  0,    0.1,   0,    1.0;
}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
