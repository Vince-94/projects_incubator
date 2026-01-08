#include "linear_mpc.hpp"

#include <unsupported/Eigen/KroneckerProduct>
#include <iostream>


using namespace mpc;


LinearMpc::LinearMpc(const StateSpace& ss, const MpcData& mpc_data, std::shared_ptr<IQpSolver> qp_solver = nullptr, bool debug)
    : nx_{ss.nx},
      nu_{ss.nu},
      ny_{ss.ny},
      mpc_data_{mpc_data} {

    if (ny_ == 0) {
        std::cout << "State-based Linear MPC" << std::endl;
    } else {
        std::cout << "Output-based Linear MPC" << std::endl;
    }

    // If no solver provided, use built-in Eigen solver
    if (!qp_solver) {
        qp_solver_ = std::make_shared<EigenQpSolver>();
    } else {
        qp_solver_ = qp_solver;
    }

    ssd_ = systemDiscretization(ss, mpc_data_.Ts, mpc_data_.discretization);

    this->checkDimensions();

    this->precomputeMatrices();

    this->precomputePrediction();

    this->initSolver(debug);
}


Eigen::VectorXd LinearMpc::onUpdate(const Eigen::VectorXd& x0, const Eigen::VectorXd& ref) {
    // 1. Prediction
    Eigen::MatrixXd X = this->prediction(x0, U_);

    // 2. Optimization
    Eigen::VectorXd U_opt = this->optimization(x0, ref);

    // 3. Receiding horizon
    u_opt_ = U_opt.head(nu_);

    // 4. Input shifting
    // U_ = U_opt[1:end]

    return u_opt_;
}


void LinearMpc::checkDimensions() {
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


void LinearMpc::precomputeMatrices() {
    std::cout << "Precompute Matrices" << std::endl;
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
}


void LinearMpc::precomputePrediction() {
// Precompute powers of A
    std::vector<Eigen::MatrixXd> A_pow(mpc_data_.Np + 1);
    A_pow[0] = Eigen::MatrixXd::Identity(nx_, nx_);  // A^0 = I
    A_pow[1] = ssd_.A;
    for (int k = 2; k <= mpc_data_.Np; ++k) {
        A_pow[k] = ssd_.A * A_pow[k - 1];
    }

    // Precompute C * A^k for k = 1 to Np (if output-based)
    std::vector<Eigen::MatrixXd> CA_pow(mpc_data_.Np + 1);
    if (ny_ > 0) {
        CA_pow[0] = Eigen::MatrixXd::Zero(ny_, nx_);  // Not used, but for consistency
        CA_pow[1] = ssd_.C * A_pow[1];
        for (int k = 2; k <= mpc_data_.Np; ++k) {
            CA_pow[k] = ssd_.C * A_pow[k];
        }
    }

    // Build the stacked prediction matrices
    for (int i = 1; i <= mpc_data_.Np; ++i) {
        // Ap_ block: A^i on the i-th row block (stacked vertically)
        int row_start_A = (i - 1) * nx_;
        Ap_.block(row_start_A, 0, nx_, nx_) = A_pow[i];  // Note: column start is 0, since Ap_ cols = nx_

        int row_start_C = (i - 1) * ny_;  // For output
        if (ny_ > 0) {
            // Cp_ block: C * A^i (stacked vertically, cols = nx_)
            Cp_.block(row_start_C, 0, ny_, nx_) = CA_pow[i];
        }

        for (int j = 1; j <= mpc_data_.Nc; ++j) {
            int col_start = (j - 1) * nu_;

            if (j <= i) {
                // Direct effect from u_{j-1|k} at step j-1 affecting step i (>= j)
                Eigen::MatrixXd temp = A_pow[i - j] * ssd_.B;
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
            Eigen::MatrixXd held_effect = A_pow[i - mpc_data_.Nc] * ssd_.B;

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


void LinearMpc::initSolver(bool /*debug*/) {
    std::cout << "Init solver" << std::endl;

    // // Create QP object
    // isize dim = mpc_data_.Nc * nu_;
    // isize n_eq = 0;
    // isize n_in = 2 * X_min_.size() + 2 * U_min_.size() + 2 * dU_min_.size()

    // qp = proxsuite::proxqp::dense::QP<double>(dim, n_eq, n_in);

    // qp.settings.verbose = debug;
    // qp.settings.eps_abs = 1e-5;      // Absolute tolerance (tune for speed vs precision)
    // qp.settings.eps_rel = 1e-6;      // Relative tolerance
    // qp.settings.max_iter = 10000;    // Max iterations
    // qp.settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;  // For receding horizon
    // qp.settings.compute_preconditioner = true;  // For ill-conditioned problems
    // qp.settings.compute_timings = true;         // Track solve time
}


Eigen::MatrixXd LinearMpc::prediction(const Eigen::VectorXd& x0, const Eigen::VectorXd& U) {
    // Check dimensions
    if (x0.size() != nx_) {
        std::string err_msg = "x0 (" + std::to_string(x0.size()) + ") must be of dimension nx: (" + std::to_string(nx_) + ")";
        throw std::invalid_argument(err_msg);
    }

    if (U.size() != nu_ * mpc_data_.Nc) {
        std::string err_msg = "U (" + std::to_string(U.size()) + ") must be of dimension nu: (" + std::to_string(nu_ * mpc_data_.Nc) + ")";
        throw std::invalid_argument(err_msg);
    }

    if (ny_ == 0) {
        Eigen::MatrixXd X = Ap_ * x0 + Bp_ * U;
        return X;
    }

    Eigen::MatrixXd Y = Cp_ * x0 + Dp_ * U;
    return Y;
}


Eigen::VectorXd LinearMpc::optimization(const Eigen::VectorXd& x0, const Eigen::VectorXd& ref) {
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

    // Number of inequality rows  // TODO precompute
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

