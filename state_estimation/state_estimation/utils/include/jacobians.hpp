#ifndef JACOBIANS_HPP
#define JACOBIANS_HPP

#include <functional>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>


/**
 * @brief Generic central finite-difference Jacobian approximation
 *
 * @tparam Func Type of the function (must return Eigen::VectorXd)
 * @param func The function to differentiate (f(x,u) or h(x,u))
 * @param x Current state vector
 * @param u Current input vector
 * @param wrt_input If true → compute wrt u (df/du or dh/du)
 *                  If false → compute wrt x (df/dx or dh/dx)
 * @param eps Finite difference step size (default 1e-8)
 * @return Jacobian matrix (n_out × n_in)
 */
template<typename Func>
Eigen::MatrixXd computeJacobian(
    const Func& func,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u,
    bool wrt_input = false,
    double eps = 1e-8)
{
    if (x.size() == 0 || u.size() == 0) {
        throw std::invalid_argument("State or input vector is empty");
    }

    // Evaluate function at nominal point (x, u)
    Eigen::VectorXd base = func(x, u);
    int n_out = base.size();
    int n_in  = wrt_input ? u.size() : x.size();

    if (n_out == 0) {
        throw std::runtime_error("Function returned empty output vector");
    }

    Eigen::MatrixXd J(n_out, n_in);
    J.setZero();

    for (int i = 0; i < n_in; ++i) {
        Eigen::VectorXd x_pert = x;
        Eigen::VectorXd u_pert = u;

        if (wrt_input) {
            u_pert(i) += eps;
        } else {
            x_pert(i) += eps;
        }

        Eigen::VectorXd perturbed = func(x_pert, u_pert);

        if (perturbed.size() != n_out) {
            throw std::runtime_error("Function output dimension changed during perturbation");
        }

        J.col(i) = (perturbed - base) / eps;
    }

    return J;
}

// Convenience overloads - more readable in user code

inline Eigen::MatrixXd jacobian_df_dx(
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& f,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u,
    double eps = 1e-8)
{
    return computeJacobian(f, x, u, false, eps);
}


inline Eigen::MatrixXd jacobian_df_du(
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& f,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u,
    double eps = 1e-8)
{
    return computeJacobian(f, x, u, true, eps);
}


inline Eigen::MatrixXd jacobian_dh_dx(
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& h,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u,
    double eps = 1e-8)
{
    return computeJacobian(h, x, u, false, eps);
}


inline Eigen::MatrixXd jacobian_dh_du(
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& h,
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& u,
    double eps = 1e-8)
{
    return computeJacobian(h, x, u, true, eps);
}


#endif  // JACOBIANS_HPP