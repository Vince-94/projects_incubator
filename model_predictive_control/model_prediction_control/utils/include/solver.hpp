#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <Eigen/Geometry>
#include <Eigen/Dense>


struct IQpSolver {
    virtual ~IQpSolver() = default;

    // Solve min 0.5 * z' H z + g' z s.t. (no constraints here for tests)
    // Return true if solved, false otherwise. solution is written to z.
    virtual Eigen::VectorXd solve(
        const Eigen::MatrixXd& H,
        const Eigen::VectorXd& f,
        const Eigen::MatrixXd& G_ineq = Eigen::MatrixXd(),  // default in base
        const Eigen::VectorXd& E_ineq = Eigen::VectorXd(),
        const Eigen::MatrixXd& G_eq   = Eigen::MatrixXd(),
        const Eigen::VectorXd& E_eq   = Eigen::VectorXd()
    ) = 0;
};


class EigenQpSolver : public IQpSolver {
public:
    Eigen::VectorXd solve(
        const Eigen::MatrixXd& H,
        const Eigen::VectorXd& f,
        const Eigen::MatrixXd& G_ineq,
        const Eigen::VectorXd& E_ineq,
        const Eigen::MatrixXd& G_eq,
        const Eigen::VectorXd& E_eq
    ) override {
        int n = H.cols();

        bool has_ineq = (G_ineq.rows() > 0 && G_ineq.cols() == n && E_ineq.size() == G_ineq.rows());
        bool has_eq   = (G_eq.rows() > 0 && G_eq.cols() == n && E_eq.size() == G_eq.rows());

        if (has_eq || has_ineq) {
            // This solver only supports unconstrained QP
            // Return empty vector as error indicator
            return Eigen::VectorXd();
        }

        // Check input dimensions
        if (H.rows() != n || f.size() != n) {
            return Eigen::VectorXd::Zero(1);  // TODO return error
        }

        // Solve unconstrained QP: min 0.5 z^T H z + f^T z
        // Solution: z = -H^{-1} f
        Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
        if (ldlt.info() != Eigen::Success) {
            return Eigen::VectorXd::Zero(1);  // TODO return error -> H not positive definite
        }

        return ldlt.solve(-f);
    }
};




#endif  // SOLVER_HPP