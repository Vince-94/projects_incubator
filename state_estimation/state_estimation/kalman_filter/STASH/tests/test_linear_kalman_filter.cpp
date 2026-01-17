#include "linear_kalman_filter.hpp"
#include "system_models/system_models.hpp"

#include <Eigen/Dense>

#include <gtest/gtest.h>


//! Fixture
class FixtureLKF : public ::testing::Test {
protected:
    void SetUp() override {
        p = 0;
        v = 5;

        init_pos_std = 2;
        init_vel_std = 10;
        acc_std = 0.1;
        gps_pos_std = 3.0;

        x0 << p, v;
        P0.diagonal() << init_pos_std, init_vel_std;
        C << 1, 0;
        Q.diagonal() << acc_std*acc_std;
        R.diagonal() << gps_pos_std*gps_pos_std;
    }

    double Ts = 0.1;
    const int nx = 2;
    const int nu = 1;
    const int ny = 1;

    double t{};
    Eigen::VectorXd x0{nx};      // initial state
    Eigen::MatrixXd P0{nx, nx};  // initial covariance matrix
    Eigen::MatrixXd C{ny, nx};   // output matrix
    Eigen::VectorXd u{nu};       // input vector
    Eigen::VectorXd z{ny};       // measurement vector
    Eigen::MatrixXd Q{nu, nu};   // incertainty matrix on input
    Eigen::MatrixXd R{ny, ny};   // incertainty matrix on sensor measurements
    Eigen::MatrixXd M{ny, ny};   //TODO unused [Eigen::MatrixXd::Identity{ny, ny};]

    // State
    double p{};   // [m]
    double v{};    // [m/s]

    // Input
    double a{};        // [m/s^2]

    // Variance
    double init_pos_std{};      // initial position standard deviation
    double init_vel_std{};      // initial velocity standard deviation
    double acc_std{};           // accelleration standard deviation
    double gps_pos_std{};       // gps position standard deviation

};



TEST_F(FixtureLKF, initialize) {
    // System model
    std::string model_name = "point_kinematic_1d_model";
    SystemModels model(model_name, Ts, x0, P0, C);

    LinearKalmanFilter lkf(std::move(model));

    // Measurements
    z << 0.0;

    // Initialization
    lkf.initialize(x0, P0);

    auto x_prio = lkf.getStatePrio();
    auto P_prio = lkf.getCovariancePrio();
    auto x_post = lkf.getStatePost();
    auto P_post = lkf.getCovariancePost();

    // Expected values
    Eigen::VectorXd x_prio_expect(nx);
    Eigen::MatrixXd P_prio_expect(nx, nx);
    x_prio_expect << 0.0, 0.0;
    P_prio_expect << 0, 0,
                     0, 0;

    Eigen::VectorXd x_post_expect(nx);
    Eigen::MatrixXd P_post_expect(nx, nx);
    x_post_expect << x0;
    P_post_expect << P0;

    EXPECT_TRUE(x_prio.isApprox(x_prio_expect, 1e-4)) << "x results\n" << x_prio.transpose() << "\nx expected\n" << x_prio_expect.transpose();
    EXPECT_TRUE(P_prio.isApprox(P_prio_expect, 1e-4)) << "P results\n" << P_prio << "\nP expected\n" << P_prio_expect;

    EXPECT_TRUE(x_post.isApprox(x_post_expect, 1e-4)) << "x results\n" << x_post.transpose() << "\nx expected\n" << x_post_expect.transpose();
    EXPECT_TRUE(P_post.isApprox(P_post_expect, 1e-4)) << "P results\n" << P_post << "\nP expected\n" << P_post_expect;
}


TEST_F(FixtureLKF, initialize_at_first_update) {
    // System model
    std::string model_name = "point_kinematic_1d_model";
    SystemModels model(model_name, Ts, x0, P0, C);

    LinearKalmanFilter lkf(std::move(model));

    // Measurements
    z << 0.0;

    // Initialize at first update
    lkf.initialize_at_first_update(z, R);

    auto x_prio = lkf.getStatePrio();
    auto P_prio = lkf.getCovariancePrio();
    auto x_post = lkf.getStatePost();
    auto P_post = lkf.getCovariancePost();

    // Expected values
    Eigen::VectorXd x_prio_expect(nx);
    Eigen::MatrixXd P_prio_expect(nx, nx);

    x_prio_expect << z(0), 0;
    P_prio_expect << R(0,0), 0,
                     0,      0;

    Eigen::VectorXd x_post_expect(nx);
    Eigen::MatrixXd P_post_expect(nx, nx);

    x_post_expect.setZero();
    P_post_expect.setZero();

    EXPECT_TRUE(x_prio.isApprox(x_prio_expect, 1e-4)) << "x results\n" << x_prio.transpose() << "\nx expected\n" << x_prio_expect.transpose();
    EXPECT_TRUE(P_prio.isApprox(P_prio_expect, 1e-4)) << "P results\n" << P_prio << "\nP expected\n" << P_prio_expect;

    EXPECT_TRUE(x_post.isApprox(x_post_expect, 1e-4)) << "x results\n" << x_post.transpose() << "\nx expected\n" << x_post_expect.transpose();
    EXPECT_TRUE(P_post.isApprox(P_post_expect, 1e-4)) << "P results\n" << P_post << "\nP expected\n" << P_post_expect;
}


TEST_F(FixtureLKF, prediction_init) {
    // System model
    std::string model_name = "point_kinematic_1d_model";
    SystemModels model(model_name, Ts, x0, P0, C);

    LinearKalmanFilter lkf(std::move(model));
    lkf.initialize(x0, P0);

    // Prediction: k = 0
    t = 0.0;        // [s]
    a = 2.0;        // [m/s^2]
    u << a;

    lkf.predict(u, Q);

    auto x_pred_0 = lkf.getStatePrio();
    auto P_pred_0 = lkf.getCovariancePrio();

    Eigen::VectorXd x_prio_expect(nx);
    Eigen::MatrixXd P_prio_expect(nx, nx);

    x_prio_expect << 0.51, 5.20;
    P_prio_expect << 2.11, 1.1,
                     1.0, 10.0;

    EXPECT_TRUE(x_pred_0.isApprox(x_prio_expect, 1e-4)) << "x results\n" << x_pred_0 << "\nx expected\n" << x_prio_expect;
    EXPECT_TRUE(P_pred_0.isApprox(P_prio_expect, 1e-4)) << "P results\n" << P_pred_0 << "\nP expected\n" << P_prio_expect;

    // Prediction: k = 0.1
    t = 0.1;        // [s]

    t = 0.0;        // [s]
    a = 2.0;        // [m/s^2]
    u << a;

    lkf.predict(u, Q);

    auto x_pred_1 = lkf.getStatePrio();
    auto P_pred_1 = lkf.getCovariancePrio();

    x_prio_expect << 1.04, 5.4;
    P_prio_expect << 2.42, 2.1,
                     2.0,  10.0002;

    EXPECT_TRUE(x_pred_1.isApprox(x_prio_expect, 1e-4)) << "x results\n" << x_pred_1 << "\nx expected\n" << x_prio_expect;
    EXPECT_TRUE(P_pred_1.isApprox(P_prio_expect, 1e-4)) << "P results\n" << P_pred_1 << "\nP expected\n" << P_prio_expect;
}



TEST_F(FixtureLKF, prediction_uninit) {
    // System model
    std::string model_name = "point_kinematic_1d_model";
    SystemModels model(model_name, Ts, x0, P0, C);

    LinearKalmanFilter lkf(std::move(model));

    // Prediction: k = 0
    t = 0.0;        // [s]
    a = 2.0;        // [m/s^2]
    u << a;

    lkf.predict(u, Q);

    auto x_pred_0 = lkf.getStatePrio();
    auto P_pred_0 = lkf.getCovariancePrio();

    Eigen::VectorXd x_prio_expect{};
    Eigen::MatrixXd P_prio_expect{};

    EXPECT_TRUE(x_pred_0.isApprox(x_prio_expect, 1e-4)) << "x results\n" << x_pred_0 << "\nx expected\n" << x_prio_expect;
    EXPECT_TRUE(P_pred_0.isApprox(P_prio_expect, 1e-4)) << "P results\n" << P_pred_0 << "\nP expected\n" << P_prio_expect;
}

TEST_F(FixtureLKF, update_init) {
    // System model
    std::string model_name = "point_kinematic_1d_model";
    SystemModels model(model_name, Ts, x0, P0, C);

    Eigen::VectorXd x0{nx};
    x0 << 1.04, 5.4;
    Eigen::MatrixXd P0{nx, nx};
    P0 << 2.42, 2.1,
              2.0,  10.0002;

    LinearKalmanFilter lkf(std::move(model));
    lkf.initialize(x0, P0);

    // Prediction: k = 0
    t = 0.0;        // [s]
    a = 2.0;        // [m/s^2]
    u << a;

    // Measurements
    z << 0.0;

    lkf.setStatePrio(x0);
    lkf.setCovariancePrio(P0);

    lkf.update(z, R);

    auto x_post = lkf.getStatePost();
    auto P_post = lkf.getCovariancePost();

    Eigen::VectorXd x_post_expect{nx};
    Eigen::MatrixXd P_post_expect{nx, nx};

    x_post_expect << 0.819614, 5.217863;
    P_post_expect << 1.907180, 1.654991,
                     1.576182, 9.632424;

    EXPECT_TRUE(x_post.isApprox(x_post_expect, 1e-4)) << "x_post results\n" << x_post << "\nx expected\n" << x_post_expect;
    EXPECT_TRUE(P_post.isApprox(P_post_expect, 1e-4)) << "P_post results\n" << P_post << "\nP expected\n" << P_post_expect;
}

TEST_F(FixtureLKF, update_uninit) {
    // System model
    std::string model_name = "point_kinematic_1d_model";
    SystemModels model(model_name, Ts, x0, P0, C);

    LinearKalmanFilter lkf(std::move(model));

    // Prediction: k = 0
    t = 0.0;        // [s]
    a = 2.0;        // [m/s^2]
    u << a;

    // Measurements
    z << 0.0;

    lkf.update(z, R);

    auto x_post = lkf.getStatePost();
    auto P_post = lkf.getCovariancePost();

    Eigen::VectorXd x_post_expect{nx};
    Eigen::MatrixXd P_post_expect{nx, nx};

    x_post_expect << 0.0, 0.0;
    P_post_expect << 4.5, 0.0,
                     0.0, 0.0;

    EXPECT_TRUE(x_post.isApprox(x_post_expect, 1e-4)) << "x_post results\n" << x_post << "\nx expected\n" << x_post_expect;
    EXPECT_TRUE(P_post.isApprox(P_post_expect, 1e-4)) << "P_post results\n" << P_post << "\nP expected\n" << P_post_expect;
}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
