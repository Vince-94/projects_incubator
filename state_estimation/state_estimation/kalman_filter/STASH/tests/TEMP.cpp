
TEST_F(FixtureLinearKalmanFilter, update_init) {
    // model at t=1
    t = 0.1;
    StateSpaceModel model = point_kinematic_2d_model.update(t);

    KalmanFilter kalman_filter{model, tuning};
    kalman_filter.initialize(x0, P0);

    // prediction data
    Eigen::VectorXd x_prio(4);
    x_prio << 0, 0, 0, 0;
    kalman_filter.setStatePrio(x_prio);

    Eigen::MatrixXd P_prio(4, 4);
    P_prio << 0.15, 0,    1.5, 0,
              0,    0.15, 0,   1.5,
              1.5,  0,    15,  0,
              0,    1.5,  0,   15;
    kalman_filter.setCovariancePrio(P_prio);

    // measurementdata
    Eigen::VectorXd z(2);
    z << 0, 0;

    kalman_filter.update(z);
    auto x_post = kalman_filter.getStatePost();
    auto P_post = kalman_filter.getCovariancePost();

    // Expected values
    Eigen::VectorXd x_post_expect(4);
    x_post_expect << 0, 0, 0, 0;

    Eigen::MatrixXd P_post_expect(4, 4);
    P_post_expect.setZero();
    P_post_expect << 0.1475, 0, 1.4754, 0,
                     0, 0.1475, 0, 1.4754,
                     1.4754, 0, 14.7540, 0,
                     0, 1.4754, 0, 14.7540;

    EXPECT_TRUE(x_post_expect.isApprox(x_post, 1e-4)) << "Vector\n" << x_post_expect << "\nnot equal to\n" << x_post;
    EXPECT_TRUE(P_post_expect.isApprox(P_post, 1e-4)) << "Matrix\n" << P_post_expect << "\nnot equal to\n" << P_post;
}


TEST_F(FixtureLinearKalmanFilter, update_uninit) {
    // model at t=1
    t = 0.1;
    StateSpaceModel model = point_kinematic_2d_model.update(t);

    KalmanFilter kalman_filter{model, tuning};

    // measurementdata
    Eigen::VectorXd z(2);
    z << 0, 1;

    kalman_filter.update(z);
    auto x_post = kalman_filter.getStatePost();
    auto P_post = kalman_filter.getCovariancePost();

    // Expected values
    Eigen::VectorXd x_post_expect(4);
    x_post_expect << 0, 1, 0, 9;

    Eigen::MatrixXd P_post_expect(4, 4);
    P_post_expect.setZero();
    P_post_expect << 4.5, 0,   0, 0,
                     0,   4.5, 0, 0,
                     0,   0,   0, 0,
                     0,   0,   0, 0;

    EXPECT_TRUE(x_post_expect.isApprox(x_post, 1e-4)) << "Vector\n" << x_post_expect << "\nnot equal to\n" << x_post;
    EXPECT_TRUE(P_post_expect.isApprox(P_post, 1e-4)) << "Matrix\n" << P_post_expect << "\nnot equal to\n" << P_post;
}


