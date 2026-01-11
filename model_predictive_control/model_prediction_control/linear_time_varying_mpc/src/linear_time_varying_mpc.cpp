#include "linear_time_varying_mpc.hpp"
#include "jacobian.hpp"

#include <unsupported/Eigen/KroneckerProduct>
#include <iostream>


using namespace mpc;


LinearTimeVaryingMpc::LinearTimeVaryingMpc(const NonlinearSystem& system_model, const MpcData& mpc_data, std::shared_ptr<IQpSolver> qp_solver = nullptr, bool debug = false)
    : system_model_{system_model},
      nx_{system_model.nx},
      nu_{system_model.nu},
      ny_{system_model.ny},
      mpc_data_{mpc_data} {

    if (ny_ == 0) {
        std::cout << "State-based Linear Time-Varying MPC" << std::endl;
    } else {
        std::cout << "Output-based Linear Time-Varying MPC" << std::endl;
    }

    // If no solver provided, use built-in Eigen solver
    if (!qp_solver) {
        qp_solver_ = std::make_shared<EigenQpSolver>();
    } else {
        qp_solver_ = qp_solver;
    }

    // Check the operating_point selection
    if (mpc_data_.operating_point > OperatingPointEnum::_Count) {
        // Fallback to OperatingPointEnum::CurrentState
        mpc_data_.operating_point = OperatingPointEnum::CurrentState;
    }

    if (mpc_data_.operating_point == OperatingPointEnum::EquilibriumPoint) {
        x_eq_ = this->computeEquilibriumPoint(system_model_, mpc_data_.x0, mpc_data_.u_eq);
        u_eq_ = mpc_data_.u_eq;
    } else if (mpc_data_.operating_point == OperatingPointEnum::PredictedTrajectory) {
        throw std::invalid_argument("PredictedTrajectory not implemented");
    }

    this->checkDimensions();

    this->precomputeMatrices();

    this->initSolver(debug);
}


Eigen::VectorXd LinearTimeVaryingMpc::computeEquilibriumPoint(
    const NonlinearSystem& nonlin_system,
    const Eigen::VectorXd& x0,
    const Eigen::VectorXd& u_eq,
    double tol,
    int max_iter
) {
    Eigen::VectorXd x = x0;
    const double eps = 1e-6;

    for (int k = 0; k < max_iter; ++k) {
        Eigen::VectorXd fx = nonlin_system.f(x, u_eq);

        if (fx.norm() < tol) {
            return x;
        }

        // Numerical Jacobian ∂f/∂x
        Eigen::MatrixXd J(nx_, nx_);
        for (int i = 0; i < nx_; ++i) {
            Eigen::VectorXd dx = Eigen::VectorXd::Zero(nx_);
            dx(i) = eps;
            J.col(i) = (nonlin_system.f(x + dx, u_eq) - fx) / eps;
        }

        // Newton step
        x -= J.fullPivLu().solve(fx);
    }

    throw std::runtime_error("Equilibrium computation did not converge");
}


void LinearTimeVaryingMpc::checkDimensions() {
    std::cout << "Check dimensions" << std::endl;
    if (mpc_data_.Ts <= 0.0) {
        std::string err_msg = "Ts (" + std::to_string(mpc_data_.Ts) + ") must be greater than 0";
        throw std::invalid_argument(err_msg);
    }

    if (mpc_data_.Np <= 0) {
        std::string err_msg = "Np (" + std::to_string(mpc_data_.Np) + ") must be a positive value";
        throw std::invalid_argument(err_msg);
    }

    if (mpc_data_.Nc > mpc_data_.Np) {
        std::string err_msg = "Np (" + std::to_string(mpc_data_.Np) + ") must be greater than Nc (" + std::to_string(mpc_data_.Nc) + ")";
        throw std::invalid_argument(err_msg);
    }

    if (ny_ == 0) {
        if (mpc_data_.Q.rows() != nx_ || mpc_data_.Q.cols() != nx_) {
            std::string err_msg = "Q must be a square matrix of size nx × nx: (" + std::to_string(nx_) + ", " + std::to_string(nx_) + ") != (" + std::to_string(mpc_data_.Q.rows()) + ", " + std::to_string(mpc_data_.Q.cols()) + ")";
            throw std::invalid_argument(err_msg);
        }
    } else {
        if (mpc_data_.Q.rows() != ny_ || mpc_data_.Q.cols() != ny_) {
            std::string err_msg = "Q must be a square matrix of size nx × nx: (" + std::to_string(ny_) + ", " + std::to_string(ny_) + ") != (" + std::to_string(mpc_data_.Q.rows()) + ", " + std::to_string(mpc_data_.Q.cols()) + ")";
            throw std::invalid_argument(err_msg);
        }
    }

    if (mpc_data_.R.rows() != nu_ || mpc_data_.R.cols() != nu_) {
        std::string err_msg = "R must be a square matrix of size nu × nu: (" + std::to_string(nu_) + ", " + std::to_string(nu_) + ") != (" + std::to_string(mpc_data_.R.rows()) + ", " + std::to_string(mpc_data_.R.cols()) + ")";
        throw std::invalid_argument(err_msg);
    }

    if (mpc_data_.S.rows() != nu_ || mpc_data_.S.cols() != nu_) {
        std::string err_msg = "S must be a square matrix of size nu × nu: (" + std::to_string(nu_) + ", " + std::to_string(nu_) + ") != (" + std::to_string(mpc_data_.S.rows()) + ", " + std::to_string(mpc_data_.S.cols()) + ")";
        throw std::invalid_argument(err_msg);
    }

    if (ny_ == 0) {
        if (mpc_data_.x_min.size() != nx_ || mpc_data_.x_max.size() != nx_) {
            std::string err_msg = "State constraints must be of dimension nx: (" + std::to_string(mpc_data_.x_min.size()) + ")";
            throw std::invalid_argument(err_msg);
        }
    } else {
        if (mpc_data_.y_min.size() != ny_ || mpc_data_.y_max.size() != ny_) {
            std::string err_msg = "Output constraints must be of dimension ny: (" + std::to_string(mpc_data_.y_min.size()) + ")";
            throw std::invalid_argument(err_msg);
        }
    }

    if (mpc_data_.u_min.size() != nu_ || mpc_data_.u_max.size() != nu_) {
        std::string err_msg = "Input constraints must be of dimension nu: (" + std::to_string(mpc_data_.u_min.size()) + ")";
        throw std::invalid_argument(err_msg);
    }

    if (mpc_data_.du_min.size() != nu_ || mpc_data_.du_max.size() != nu_) {
        std::string err_msg = "Input rate constraints must be of dimension nu: (" + std::to_string(mpc_data_.du_min.size()) + ")";
        throw std::invalid_argument(err_msg);
    }
}


void LinearTimeVaryingMpc::precomputeMatrices() {
    // Prediction matrices
    Ap_ = Eigen::MatrixXd::Zero(mpc_data_.Np * nx_, nx_);
    Bp_ = Eigen::MatrixXd::Zero(mpc_data_.Np * nx_, mpc_data_.Nc * nu_);
    if (ny_ > 0) {
        Cp_ = Eigen::MatrixXd::Zero(mpc_data_.Np * ny_, nx_);
        Dp_ = Eigen::MatrixXd::Zero(mpc_data_.Np * ny_, mpc_data_.Nc * nu_);
    }

    // Input vector
    u_opt_ = Eigen::VectorXd::Zero(nu_);

    // Input stack
    U_ = Eigen::VectorXd::Zero(mpc_data_.Nc * nu_);

    // Input previous stack: U_stacked_ = [u_{k-1}; 0; 0; ...]
    U_stacked_ = Eigen::VectorXd::Zero(mpc_data_.Nc * nu_);

    // M matrix
    M_ = Eigen::MatrixXd::Identity(mpc_data_.Nc * nu_, mpc_data_.Nc * nu_);
    for (int i = 1; i < mpc_data_.Nc; ++i) {
        int row_start = i * nu_;
        int col_start = (i - 1) * nu_;
        M_.block(row_start, col_start, nu_, nu_) -= Eigen::MatrixXd::Identity(nu_, nu_);
    }

    // Q_ = blkdiag(Q, Q, ..., Q), (Np times) → size (Np * ny, Np * ny)
    Qp_ = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(mpc_data_.Np, mpc_data_.Np), mpc_data_.Q).eval();

    // Rp_ = blkdiag(R, R, ..., R), (Nc times) → size (Nc * nu, Nc * nu)
    Rp_ = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(mpc_data_.Nc, mpc_data_.Nc), mpc_data_.R).eval();

    // Sp_ = blkdiag(S, S, ..., S), (Nc times) → size (Nc * nu, Nc * nu)
    Sp_ = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(mpc_data_.Nc, mpc_data_.Nc), mpc_data_.S).eval();

    if (ny_ == 0) {
        // X_min, X_max → size (Np * nx, 1)
        X_min_ = mpc_data_.x_min.replicate(mpc_data_.Np, 1);
        X_max_ = mpc_data_.x_max.replicate(mpc_data_.Np, 1);
    } else {
        // Y_min, Y_max → size (Np * nx, 1)
        Y_min_ = mpc_data_.y_min.replicate(mpc_data_.Np, 1);
        Y_max_ = mpc_data_.y_max.replicate(mpc_data_.Np, 1);
    }

    // U_min, U_max
    U_min_ = mpc_data_.u_min.replicate(mpc_data_.Nc, 1);
    U_max_ = mpc_data_.u_max.replicate(mpc_data_.Nc, 1);

    // dU_min, dU_max
    dU_min_ = mpc_data_.du_min.replicate(mpc_data_.Nc, 1);
    dU_max_ = mpc_data_.du_max.replicate(mpc_data_.Nc, 1);

    // A_pow_
    A_pow_.resize(mpc_data_.Np + 1);
    A_pow_[0] = Eigen::MatrixXd::Identity(nx_, nx_);  // A^0 = I

    // CA_pow_
    if (ny_ > 0) {
        CA_pow_.resize(mpc_data_.Np + 1);
        CA_pow_[0] = Eigen::MatrixXd::Zero(ny_, nx_);  // TODO useless
    }
}


void LinearTimeVaryingMpc::initSolver(bool /*debug*/) {
}


Eigen::VectorXd LinearTimeVaryingMpc::onUpdate(const Eigen::VectorXd& x0, const Eigen::VectorXd& ref) {
    // 1. Operating point
    this->getOperatingPoint();

    // 2. Linearization
    StateSpace ss = this->linearization(system_model_, op_point_.x, op_point_.u);

    // 3. Discretizetion
    ssd_ = systemDiscretization(ss, mpc_data_.Ts, mpc_data_.discretization);

    // 4. Prediction
    Eigen::MatrixXd X = this->prediction(x0, U_);

    // Optimization
    U_ = this->optimization(x0, ref);

    // Receiding horizon to the first input
    u_opt_ = U_.head(nu_);

    // Input shifting for warm-start
    Eigen::VectorXd last_values = U_.tail(nu_);
    U_.head(mpc_data_.Nc * nu_ - nu_) = U_.segment(nu_, mpc_data_.Nc * nu_ - nu_);
    U_.tail(nu_) = last_values;
    // U_.tail(nu_) = U_.segment(mpc_data_.Nc * nu_ - 2 * nu_, nu_);  // TODO slightly more performant

    return u_opt_;
}


void LinearTimeVaryingMpc::getOperatingPoint() {
    Eigen::VectorXd xo, uo;

    if (mpc_data_.operating_point == OperatingPointEnum::EquilibriumPoint) {
        // x_o is the value for which: x_dot = f(x_o) = 0
        xo = x_eq_;
        uo = u_eq_;
    } else if (mpc_data_.operating_point == OperatingPointEnum::CurrentState) {
        // Operating point is the actual system state
        xo = x0_;
        uo = u_opt_;
    } else if (mpc_data_.operating_point == OperatingPointEnum::PredictedTrajectory) {
        // TODO
        // if (first_call_) {
        //     // Initial: use zero U
        // } else if (mpc_data_.Nc > 1) {
        //     U_bar.head((mpc_data_.Nc - 1) * nu_) = U_pred_.segment(nu_, (mpc_data_.Nc - 1) * nu_);
        // }
        // // Propagate nominal using nonlinear model
        // x_traj.resize(mpc_data_.Np + 1);
        // u_traj.resize(mpc_data_.Np);
        // x_traj[0] = x0;
        // for (int k = 0; k < mpc_data_.Np; ++k) {
        //     int uk_idx = std::min(k, mpc_data_.Nc - 1);
        //     u_traj[k] = U_bar.segment(uk_idx * nu_, nu_);
        //     x_traj[k + 1] = rk4(x_traj[k], u_traj[k], mpc_data_.Ts);
        // }
    }

    op_point_.x = xo;
    op_point_.u = uo;
}


Eigen::VectorXd LinearTimeVaryingMpc::rk4(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double Ts) {
    auto dx = [this, &u](const Eigen::VectorXd& state) {
        return system_model_.f(state, u);
    };

    Eigen::VectorXd k1 = dx(x);
    Eigen::VectorXd k2 = dx(x + Ts * 0.5 * k1);
    Eigen::VectorXd k3 = dx(x + Ts * 0.5 * k2);
    Eigen::VectorXd k4 = dx(x + Ts * k3);

    return x + (Ts / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}


StateSpace LinearTimeVaryingMpc::linearization(const NonlinearSystem& system_model, const Eigen::VectorXd& x0, const Eigen::VectorXd& u0) {
    // Compute A = ∂f/∂x
    auto A = jacobian_df_dx(system_model.f, x0, u0);

    // Compute B = ∂f/∂u
    auto B = jacobian_df_du(system_model.f, x0, u0);

    // Compute C = ∂g/∂x
    auto C = jacobian_dh_dx(system_model.h, x0, u0);

    // Compute D = ∂g/∂u
    auto D = jacobian_dh_du(system_model.h, x0, u0);

    return StateSpace{A, B, C, D};
}


Eigen::MatrixXd LinearTimeVaryingMpc::prediction(const Eigen::VectorXd& x0, const Eigen::VectorXd& U) {
    // Check x0 dimensions
    if (x0.size() != nx_) {
        std::string err_msg = "x0 (" + std::to_string(x0.size()) + ") must be of dimension nx: (" + std::to_string(nx_) + ")";
        throw std::invalid_argument(err_msg);
    }

    // Check U dimensions
    if (U.size() != nu_ * mpc_data_.Nc) {
        std::string err_msg = "U (" + std::to_string(U.size()) + ") must be of dimension nu: (" + std::to_string(nu_ * mpc_data_.Nc) + ")";
        throw std::invalid_argument(err_msg);
    }

    // Compute prediction matrices
    this->computePredictionMatrices();

    if (ny_ == 0) {
        Eigen::MatrixXd X = Ap_ * x0 + Bp_ * U;
        return X;
    }

    Eigen::MatrixXd Y = Cp_ * x0 + Dp_ * U;
    return Y;
}


void LinearTimeVaryingMpc::computePredictionMatrices() {
    // Precompute powers of A
    A_pow_[1] = ssd_.A;
    for (int k = 2; k <= mpc_data_.Np; ++k) {
        A_pow_[k] = ssd_.A * A_pow_[k - 1];
    }

    // Precompute C * A^k for k = 1 to Np (if output-based)
    if (ny_ > 0) {
        CA_pow_[1] = ssd_.C * A_pow_[1];
        for (int k = 2; k <= mpc_data_.Np; ++k) {
            CA_pow_[k] = ssd_.C * A_pow_[k];
        }
    }

    // Build the stacked prediction matrices
    for (int i = 1; i <= mpc_data_.Np; ++i) {
        // Ap_ block: A^i on the i-th row block (stacked vertically)
        int row_start_A = (i - 1) * nx_;
        Ap_.block(row_start_A, 0, nx_, nx_) = A_pow_[i];  // Note: column start is 0, since Ap_ cols = nx_

        int row_start_C = (i - 1) * ny_;  // For output
        if (ny_ > 0) {
            // Cp_ block: C * A^i (stacked vertically, cols = nx_)
            Cp_.block(row_start_C, 0, ny_, nx_) = CA_pow_[i];
        }

        for (int j = 1; j <= mpc_data_.Nc; ++j) {
            int col_start = (j - 1) * nu_;

            if (j <= i) {
                // Direct effect from u_{j-1|k} at step j-1 affecting step i (>= j)
                Eigen::MatrixXd temp = A_pow_[i - j] * ssd_.B;
                Bp_.block(row_start_A, col_start, nx_, nu_) = temp;

                if (ny_ > 0) {
                    Eigen::MatrixXd temp_dyn = ssd_.C * temp;
                    if (i == j) temp_dyn += ssd_.D;
                    Dp_.block(row_start_C, col_start, ny_, nu_) = temp_dyn;
                }
            }
        }

        // Add the effect of held input after Nc (if i > Nc)
        if (i > mpc_data_.Nc) {
            // The last input u_{Nc-1|k} affects all future steps via A^{i - Nc} * B
            Eigen::MatrixXd held_effect = A_pow_[i - mpc_data_.Nc] * ssd_.B;

            // Add it to the LAST column of U (corresponding to u_{Nc-1})
            int last_col = (mpc_data_.Nc - 1) * nu_;
            Bp_.block(row_start_A, last_col, nx_, nu_) += held_effect;

            if (ny_ > 0) {
                Eigen::MatrixXd held_dyn = ssd_.C * held_effect + ssd_.D;  // direct + dynamic
                Dp_.block(row_start_C, last_col, ny_, nu_) += held_dyn;
            }
        }
    }
}


Eigen::VectorXd LinearTimeVaryingMpc::optimization(const Eigen::VectorXd& x0, const Eigen::VectorXd& ref) {
    if (x0.size() != nx_) {
        std::string err_msg = "x0 (" + std::to_string(x0.size()) + ") must be of dimension nx: (" + std::to_string(nx_) + ")";
        throw std::invalid_argument(err_msg);
    }

    if (ny_ == 0 && ref.size() != nx_) {
        std::string err_msg = "ref (" + std::to_string(ref.size()) + ") must be of dimension nx: (" + std::to_string(nx_) + ")";
        throw std::invalid_argument(err_msg);
    } else if (ny_ > 0 && ref.size() != ny_) {
        std::string err_msg = "ref (" + std::to_string(ref.size()) + ") must be of dimension ny: (" + std::to_string(ny_) + ")";
        throw std::invalid_argument(err_msg);
    }

    //* QP matrices
    // H = Bp^T Qp Bp + Rp + M^T Sp M
    // f(k) = Bp^T Qp * error - M^T Sp U_prev

    // Reference stack
    Eigen::VectorXd REF = ref.replicate(mpc_data_.Np, 1);
    Eigen::VectorXd error;
    Eigen::MatrixXd H;
    Eigen::VectorXd f;

    // Tracking error
    if (ny_ == 0 ) {
        error = Ap_ * x0 - REF;
        H = Bp_.transpose() * Qp_ * Bp_ + Rp_;
        f = Bp_.transpose() * Qp_ * error;
    } else {
        error = Cp_ * x0 - REF;
        H = Dp_.transpose() * Qp_ * Dp_ + Rp_;
        f = Dp_.transpose() * Qp_ * error;
    }

    if (Sp_.squaredNorm() > 0) {
        H += M_.transpose() * Sp_ * M_;
        U_stacked_.head(nu_) = u_opt_;
        f -= M_.transpose() * Sp_ * U_stacked_;
    }

    //* Build inequality constraints: G U <= E(k)
    // -Bp U <= -X_min + Ap x0
    //  Bp U <=  X_max - Ap x0
    // -I  U <= -U_min
    //  I  U <=  U_max
    // -M  U <= -dU_min - U_stacked_
    //  M  U <=  dU_max - U_stacked_

    // Number of inequality rows
    int n_ineq;
    // State/output constraint number
    if (ny_ == 0) {
        n_ineq = 2 * (mpc_data_.Np * nx_);
    } else {
        n_ineq = 2 * (mpc_data_.Np * ny_);
    }

    // Input constraint number
    n_ineq += 2 * (mpc_data_.Nc * nu_);

    // Input rate constraint number
    if (Sp_.squaredNorm() > 0) {
        n_ineq += 2 * (mpc_data_.Nc * nu_);
    }

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(n_ineq, mpc_data_.Nc * nu_);
    Eigen::VectorXd E = Eigen::VectorXd::Zero(n_ineq);

    int row = 0;

    // State constraints
    if (ny_ == 0) {
        G.middleRows(row, mpc_data_.Np * nx_) = -Bp_;
        E.segment(row, mpc_data_.Np * nx_) = -X_min_ + Ap_ * x0;
        row += mpc_data_.Np * nx_;

        G.middleRows(row, mpc_data_.Np * nx_) = Bp_;
        E.segment(row, mpc_data_.Np * nx_) = X_max_ - Ap_ * x0;
        row += mpc_data_.Np * nx_;
    } else {
        G.middleRows(row, mpc_data_.Np * ny_) = -Dp_;
        E.segment(row, mpc_data_.Np * ny_) = -Y_min_ + Cp_ * x0;
        row += mpc_data_.Np * ny_;

        G.middleRows(row, mpc_data_.Np * ny_) = Dp_;
        E.segment(row, mpc_data_.Np * ny_) = Y_max_ - Cp_ * x0;
        row += mpc_data_.Np * ny_;
    }

    // Input bounds
    G.middleRows(row, mpc_data_.Nc * nu_) = -Eigen::MatrixXd::Identity(mpc_data_.Nc * nu_, mpc_data_.Nc * nu_);
    E.segment(row, mpc_data_.Nc * nu_) = -U_min_;
    row += mpc_data_.Nc * nu_;

    G.middleRows(row, mpc_data_.Nc * nu_) = Eigen::MatrixXd::Identity(mpc_data_.Nc * nu_, mpc_data_.Nc * nu_);
    E.segment(row, mpc_data_.Nc * nu_) = U_max_;
    row += mpc_data_.Nc * nu_;

    // Input rate constraints
    if (Sp_.squaredNorm() > 0) {
        G.middleRows(row, mpc_data_.Nc * nu_) = -M_;
        E.segment(row, mpc_data_.Nc * nu_) = -dU_min_ - U_stacked_;
        row += mpc_data_.Nc * nu_;

        G.middleRows(row, mpc_data_.Nc * nu_) = M_;
        E.segment(row, mpc_data_.Nc * nu_) = dU_max_ - U_stacked_;
        row += mpc_data_.Nc * nu_;
    }

    //* Solve QP: min 0.5 U^T (2H) U + (2f)^T U → but we use standard form → H U + f = 0 → U_opt = -H^{-1} f
    Eigen::VectorXd U_opt = qp_solver_->solve(H, f, G, E);

    if (U_opt.size() == 0) {
        std::cerr << "QP failed or constraints not supported" << std::endl;
        return U_opt;  // fallback
    }

    // Return first control input
    return U_opt;
}


