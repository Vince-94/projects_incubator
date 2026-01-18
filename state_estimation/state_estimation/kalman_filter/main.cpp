#include "linear_kalman_filter.hpp"

#include <iostream>
#include <iomanip>
#include <vector>

#include <Eigen/Dense>


double dt = 0.1;  // TODO


// ────────────────────────────────────────────────────────────────
// Very simple 1D constant velocity model (position + velocity)
// State:       x = [position, velocity]^T
// Control:     u = acceleration (we set to zero here)
// Measurement: z = position only
// ────────────────────────────────────────────────────────────────

struct ConstVel1dModel {

    Eigen::MatrixXd A{
        {1.0, dt},
        {0.0, 1.0}
    };
    Eigen::MatrixXd B{
        {dt*dt/2.0},
        {dt}
    };
    Eigen::MatrixXd C{
        {1.0, 0.0}
    };

    Eigen::VectorXd updateState(Eigen::VectorXd x0, Eigen::VectorXd u) {
        // position += velocity * dt;
        auto pos = x0(0) + x0(1) * dt + u(0) * dt * dt;
        auto vel = x0(1);
        Eigen::VectorXd x{
            {pos, vel}
        };
        return x;
    }
};




int main() {
    std::cout << "=== Simple 1D Constant Velocity Kalman Filter Test ===" << std::endl;

    // ───────────────────────────────────────────────
    // 1. Create system model
    // ───────────────────────────────────────────────
    ConstVel1dModel const_vel_1d{};

    StateSpace sys_model(const_vel_1d.A, const_vel_1d.B, const_vel_1d.C);

    // Process & measurement noise
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(2, 2);
    Q.diagonal() << 0.01, 0.001;               // small process noise

    Eigen::MatrixXd R(1, 1);
    R(0,0) = 4.0;                              // measurement std = 0.5 m

    // ───────────────────────────────────────────────
    // 2. Create filter
    // ───────────────────────────────────────────────
    LinearKalmanFilter lkf(std::move(sys_model));


    // ───────────────────────────────────────────────
    // 3. Initialize
    // ───────────────────────────────────────────────
    Eigen::VectorXd x0{
        {0.0, 1.0}
    };
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(2, 2);
    P0.diagonal() << 20.0, 20.0;

    lkf.initialize(x0, P0);


    // ───────────────────────────────────────────────
    // 4. Simulation parameters
    // ───────────────────────────────────────────────
    // const int sim_time = 10;  // [s]
    // const int n_steps = sim_time * dt;  // loop steps
    const int n_steps = 100;  // loop steps

    // State initialization
    double position = 0.0;  // x[0]
    double velocity = 1.0;  // x[1]
    Eigen::VectorXd x = Eigen::VectorXd::Zero(2);
    x << position, velocity;

    // Input initialization
    Eigen::VectorXd u = Eigen::VectorXd::Zero(1);

    // Control params
    std::vector<double> pos_v;  // position
    std::vector<double> vel_v;  // velocity
    std::vector<double> pos_est_v;  // position
    std::vector<double> vel_est_v;  // velocity
    std::vector<double> pos_cov_v;  // position variance
    std::vector<double> vel_cov_v;  // velocity variance


    // ───────────────────────────────────────────────
    // 5. Simulation loop
    // ───────────────────────────────────────────────

    for (int k = 0; k < n_steps; ++k) {
        // Update model state
        x = const_vel_1d.updateState(x, u);

        // Simulate noisy measurement
        double meas_noise = 0.5 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2.0;

        Eigen::VectorXd z(1);
        z(0) = x(0) + meas_noise;

        // Kalman filter steps
        lkf.predict(u, Q);
        lkf.update(z, R);

        // Get estimate
        Eigen::VectorXd state_est = lkf.getStatePost();
        Eigen::MatrixXd cov_est = lkf.getCovariancePost();

        // Push data
        pos_v.push_back(x(0));
        pos_est_v.push_back(state_est(0));
        pos_cov_v.push_back(cov_est(0));
    }


    // ───────────────────────────────────────────────
    // Optional: simple console plot (very basic)
    // ───────────────────────────────────────────────
    std::cout << "\nBasic position plot (T = true, E = estimate, . = 1 sigma bound)\n";
    for (size_t i = 0; i < pos_v.size(); ++i) {
        if (i % 5 != 0) continue;
        double p = pos_v[i];
        double e = pos_est_v[i];
        double s = std::sqrt(pos_cov_v[i]);

        std::cout << std::setw(4) << i*dt << "s | ";
        int width = 60;
        int true_pos  = static_cast<int>((p + 5) / 10.0 * width);
        int est_pos   = static_cast<int>((e + 5) / 10.0 * width);
        int lower     = static_cast<int>((e - s + 5) / 10.0 * width);
        int upper     = static_cast<int>((e + s + 5) / 10.0 * width);

        for (int c = 0; c < width; ++c) {
            if (c == true_pos)  std::cout << "T";
            else if (c == est_pos) std::cout << "E";
            else if (c >= lower && c <= upper) std::cout << ".";
            else std::cout << " ";
        }
        std::cout << "\n";
    }



    return 0;
}
