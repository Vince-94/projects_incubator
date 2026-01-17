#include "system_models.hpp"

#include <Eigen/Dense>

#include <gtest/gtest.h>


TEST(PointKinematic2DModel, TestOnStep) {
    PointKinematic2DModel point_kinematic_2d_model{};
    int nx = point_kinematic_2d_model.getNx();
    int nu = point_kinematic_2d_model.getNu();

    Eigen::VectorXd x0{nx};
    x0 << 0, 0, 0, 0;

    Eigen::MatrixXd P0{nx, nx};
    P0 << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

    Eigen::VectorXd u{nu};
    u << 0, 0;

    Eigen::MatrixXd Q{nu, nu};
    Q << 1, 0,
         0, 1;

    double t = 0.0;

    point_kinematic_2d_model.on_step(x0, P0, u, Q, t);

    auto x = point_kinematic_2d_model.getState();
    auto P = point_kinematic_2d_model.getCovariance();

    // Expected
    Eigen::VectorXd x_expect{nx};
    x_expect << 0, 0, 0, 0;

    Eigen::MatrixXd P_expect{nx, nx};
    P_expect << 0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;

    EXPECT_TRUE(x.isApprox(x_expect, 1e-4)) << "x result:\n" << x << "\nx expected\n" << x_expect;
    EXPECT_TRUE(P.isApprox(P_expect, 1e-4)) << "P result:\n" << P << "\nP expected\n" << P_expect;
}


TEST(Vehicle2DModel, TestOnStep) {
    Vehicle2DModel vehicle_2d_model{};
    int nx = vehicle_2d_model.getNx();
    int nu = vehicle_2d_model.getNu();

    Eigen::VectorXd x0{nx};
    x0 << 0, 0, 0, 0;

    Eigen::MatrixXd P0{nx, nx};
    P0 << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

    Eigen::VectorXd u{nu};
    u << 0, 0;

    Eigen::MatrixXd Q{nu, nu};
    Q << 1, 0,
         0, 1;

    double t = 0.0;

    vehicle_2d_model.on_step(x0, P0, u, Q, t);

    auto x = vehicle_2d_model.getState();
    auto P = vehicle_2d_model.getCovariance();

    // Expected
    Eigen::VectorXd x_expect{nx};
    x_expect << 0, 0, 0, 0;

    Eigen::MatrixXd P_expect{nx, nx};
    P_expect << 1, 0, 1, 0,
                0, 1, 0, 1,
                1, 0, 1, 0,
                0, 1, 0, 1;

    EXPECT_TRUE(x.isApprox(x_expect, 1e-4)) << "x result:\n" << x << "\nx expected\n" << x_expect;
    EXPECT_TRUE(P.isApprox(P_expect, 1e-4)) << "P result:\n" << P << "\nP expected\n" << P_expect;
}



int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
