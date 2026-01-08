# Model Predictive Control

- [Model Predictive Control](#model-predictive-control)
  - [Workflow](#workflow)
  - [Design](#design)
  - [State Prediction](#state-prediction)
  - [Optimization Problem](#optimization-problem)
  - [Tuning parameters](#tuning-parameters)
    - [Prediction/Control Horizon](#predictioncontrol-horizon)
  - [Variants](#variants)
  - [Appendix](#appendix)
    - [Discretization](#discretization)
      - [Euler approximation](#euler-approximation)
    - [Terminal components](#terminal-components)
      - [Terminal cost](#terminal-cost)
      - [Terminal state constraint (invariant terminal set or safe set)](#terminal-state-constraint-invariant-terminal-set-or-safe-set)


## Workflow

At k-th sampling time:
1. Predict the system future state over the prediction horizon
2. Solve QP (Quadratic Program) optimization problem
3. Check feasibility, stability, robustness, performance and real-time
4. Receiding horizon
5. Wait the k+1 sampling time and repeat from point 1


## Design

- System modelling
- Optimization problem definition (cost function + constraints)
- State observer designing



## State Prediction


$$
\begin{aligned}
    \dot{x}(t) &= f(x, u), \qquad x(0) = x(t=0) \\
    y(t) &= g(x, u)
\end{aligned}
\quad
\rightarrow
\quad
\begin{aligned}
    X(t) &= \begin{bmatrix} x_0 & x_1 & \cdots & x_{N_p-1} \end{bmatrix}^\top \in \mathbb{R}^{n_x N_p} \\
    Y(t) &= \begin{bmatrix} y_0 & y_1 & \cdots & y_{N_p-1} \end{bmatrix}^\top \in \mathbb{R}^{n_y N_p}
\end{aligned}
$$

where:
- $\dot{x}(t) \in \mathbb{R}^{n_x} \quad \text{future state vector}$
- $x(t) \in \mathbb{R}^{n_x} \quad \text{state vector}$
- $u(t) \in \mathbb{R}^{n_u} \quad \text{input vector}$
- $y(t) \in \mathbb{R}^{n_y} \quad \text{output vector}$


## Optimization Problem

$$
\begin{aligned}
    &\min_{U} && J(x(k), U) \\
    &\text{s.t.} && A_{\text{ineq}}(k) \, U_k \leq b_{\text{ineq}}(k) \\
    &            && A_{\text{eq}}(k) \, U_k = b_{\text{eq}}(k)
\end{aligned}
$$

$$
\begin{aligned}
    &\min_{U_k} && \sum_{k=0}^{N-1} \ell(x_{k}, u_{k}) &&& \text{cost function} \\
    &\text{s.t.} && x_{k+1} = f(x_{k}, u_{k}), \qquad k = 0, \dots, N-1 &&& \text{system model} \\
    & && x_{0} = x(0) &&& \text{initial state} \\
    & && x_{k} \in \mathcal{X}, \qquad k = 0, \dots, N-1 &&& \text{state constraint} \\
    & && u_{k} \in \mathcal{U}, \qquad k = 0, \dots, N-1 &&& \text{input constraint} \\
    & && \Delta u_{k} \in \mathcal{U}, \qquad k = 0, \dots, N-1 &&& \text{input rate constraint} \\
\end{aligned}
$$

where:
- $N$ prediction/control horizon
- $\ell$ stage cost
- $\mathcal{X}, \mathcal{U}$ state and input constaint sets

The solution is the optimal input sequence:
$$
    U^* = \begin{bmatrix} u^*_{0} & u^*_{1} & \cdots & u^*_{N-1} \end{bmatrix} \in \mathbb{R}^{n_u N}
$$

Due to the receding-horizon law, apply to the system the first input of the sequence:
$$
    u(k) = u^*_{0}
$$

It is possible to use the shifted $U^*$ as warm-start:
$$
    U^*_0 = \begin{bmatrix} u^*_{1} & u^*_{2} & \cdots & u^*_{N-1} & u^*_{N-1} \end{bmatrix} \in \mathbb{R}^{n_u N}
$$





## Tuning parameters

### Prediction/Control Horizon

The `prediction horizon` $N_p$ is the number of time steps into the future over which the MPC controller predicts the evolution of the system states/outputs.
The control horizon `control horizon` is the number of future time steps over which the MPC controller explicitly optimizes the control inputs.

When $N_c < N_p$, inputs are held constant after $N_c$: $u_{i|k} = u_{N_c-1|k}$ for $i = N_c, \dots, N_p-1$. This ensures robust predictions without assuming inputs drop to zero, which could destabilize the system.

The control horizon is always:
$$
N_c \leq N_p
$$
Advantages:
- Computational efficiency: Dramatically reduces the size of the QP, making real-time solution much faster.
- Better numerical conditioning: Optimizing fewer variables reduces ill-conditioning in the QP, especially when $N_p$ is large
- Sufficient for good closed-loop performance: A shorter control horizon still allows the controller to "steer" the system toward the desired reference over the longer prediction horizon.
- Practical engineering trade-off: In many applications (e.g., automotive, process control), a long prediction horizon is needed for stability and constraint satisfaction, but optimizing every single step is computationally too expensive.



## Variants

- Linear MPC: The system is linear and time-invariant expressed in its state space. The OCP becomes a quadratic program (QP), convex and efficient.
- Parameter-varying MPC: The system is linear and parameter-varying expressed in its state space. The OCP becomes a quadratic program (QP), convex and efficient.
- Time-varying MPC: The system is linear and time-varying expressed in its state space. The OCP becomes a quadratic program (QP), convex and efficient.
- Nonlinear MPC: The system is nonlinear. The OCP is a non-convex nonlinear program (NLP), solved iteratively. Stability often requires terminal constraints/penalties.
- Stochastic MPC: Handles probabilistic uncertainty (e.g., stochastic disturbances $  w_k  $ with known distributions). Instead of deterministic constraints, it uses chance constraints (probabilistic guarantees) and minimizes an expected cost.
- Tube-based MPC: Handles uncertainty (e.g., additive bounded disturbances). It uses a nominal system to keep real trajectories in a "tube" around the nominal path (parameterized by robust positively invariant sets).
- Robust min-max MPC: Optimizes for the worst-case scenario over bounded uncertainty sets. It uses a min-max objective to minimize the maximum cost over all possible disturbances.
- Hybrid MPC: For systems with both continuous and discrete dynamics (e.g., mixed logical dynamical models, piecewise affine). Incorporates integer/binary variables for modes/logic. The OCP becomes a mixed-integer program (MIP), solved via MIQP/MINLP. Stability via terminal sets invariant under modes.
- Learning-based MPC (LB-MPC): Integrates ML to learn system dynamics or uncertainties directly from data, replacing or augmenting first-principles models. Common ML tools include Gaussian Processes (GPs) for probabilistic modeling (capturing mean predictions and uncertainty bounds) and Neural Networks (NNs, e.g., RNNs/LSTMs) for nonlinear approximations. The OCP uses the learned model for predictions.
- Data-driven MPC: Purely relies on historical/input-output data without explicit parametric models. Sub-variants include behavioral theory (using Hankel matrices of past trajectories via Willems' fundamental lemma) and ML-based (e.g., GPs/NNs for direct prediction). The OCP is formulated over persisted trajectories or learned surrogates, often with robust constraints tightened by uncertainty estimates.
- Iterative Learning MPC: Combines MPC with iterative learning control (ILC) for repetitive tasks (e.g., batch processes), where the system restarts from the same initial state.
- Hybrid MPC-RL: Combines MPC's constraint handling and optimality with RL's exploration and long-horizon learning. RL tunes MPC parameters (e.g., costs/weights), generates policies, or approximates value functions/terminal costs; MPC provides safe rollouts or robust backups. Frameworks view MPC as a policy improvement operator in dynamic programming, with RL for offline training.



## Appendix

### Discretization

#### Euler approximation
$$
\begin{aligned}
    A_d &= I_{n_x} + A \, T_s \\
    B_d &= B \, T_s \\
    C_d &= C \\
    D_d &= D
\end{aligned}
$$


### Terminal components

Terminal components (cost and constraint) are meant to "approximate" cost and constraint beyond the prediction horizon (as if it were an infinit LQR), otherwise the shortsighted control actions could lead to a state from which it is impossible remain within the state constraint set (infeasibility).

$$
\begin{aligned}
    &\min_{U_k} && V_f(x) + \sum_{k=0}^{N-1} \ell(x_{k}, u_{k}) &&& \text{cost function} \\
    &\text{s.t.} && x_{k+1} = f(x_{k}, u_{k}), \qquad k = 0, \dots, N-1 &&& \text{system model} \\
    & && x_{0} = x(0) &&& \text{initial state} \\
    & && x_{k} \in \mathcal{X}, \qquad k = 0, \dots, N-1 &&& \text{state constraint} \\
    & && u_{k} \in \mathcal{U}, \qquad k = 0, \dots, N-1 &&& \text{input constraint} \\
    & && \Delta u_{k} \in \mathcal{U}, \qquad k = 0, \dots, N-1 &&& \text{input rate constraint} \\
    & && x_{N} \in \mathcal{X}_f &&& \text{terminal constraint}
\end{aligned}
$$

where:
- $V_f(x)$ terminal (Mayer) cost
- $\mathcal{X}_f$ terminal set

This introduces a local feedback controller $u = K x$ that stabilizes the system inside $\mathcal{X}_f$.


#### Terminal cost
The terminal cost approximates the infinite-horizon cost beyond $N_p$, ensuring stability of the closed-loop system.

$$
\begin{aligned}
    J_{term} &= \| x_{N_p|k} \|_P^2
\end{aligned}
$$

where $P⪰0$ is typically chosen as the solution of the discrete-time Algebraic Riccati Equation (LQR) for ($A_d$, $B_d$, $Q$, $R$):
$$
P = A_d^T \, P \, A_d - A_d^T \, P \, B_d \, (R + B_d^T \, P \, B_d)^{-1} \, B_d^T \, P \, A_d + Q
$$

The LQR gain is:
$$
K = - (R + B_d^T \, P \, B_d)^{-1} \, B_d^T \, P \, A_d
$$

The closed-loop matrix is:
$$
A_{cl} = A_d + B_d \, K
$$

This ensures the cost decreases inside $\mathcal{X}_f$, implying stability.


#### Terminal state constraint (invariant terminal set or safe set)

The terminal constraint set guarantees closed-loop stability, recursive feasibility (if the QP is feasible now, it will remain feasible in future steps), and good performance with finite prediction horizons.

$$
x_{N_p|k} \in \mathcal{X}_f
$$

Where: $\mathcal{X}_f = \{ x : \|x - r_{ref}\|_{Q_f} \leq \epsilon \} \qquad \text{robust positive invariant set for the terminal controller} \quad u=Kx$

In practice for each state $x \in \mathcal{X}_f$, there exists an admissible input $u = K x$ such that:
- the next state $x^+ = A_d x + B_d (K x)$ remains in $\mathcal{X}_f$
- all constraints (state and input) are satisfied

The $\mathcal{X}_f$ is represented as a polyhedron: $\mathcal{X}_f = \{ x \, | \, F_f \, x \leq f_f \}$
The $\mathcal{X}_f = \{ x \, | \, F_f \, x \leq f_f \}$ is known as the `Polyhedral (H-representation) terminal set`, often the maximal control-invariant set (largest possible invariant set under constraints).

It can be expressed as:
$$
F_f \, x_{N_p|k} \leq f_f \quad \rightarrow \quad F_f \, (E_N \, \bar{A} + E_N \, \bar{B} \, U_k) \leq f_f  \quad \Rightarrow \quad (F_f \, E_N \, \bar{B}) \, U_k\leq f_f - F_f \, E_N \, \bar{A}
$$

where:
- $x_{N_p|k} = E_N \, X_k = E_N \, (\bar{A} \, x_k + \bar{B} \, U_k)$
- $E_N = [0_{n_x,n_x(N_p-1)} \ I_{n_x}] \in \mathbb{R}^{n_x, n_x N_p} \quad \text{selection matrix extracting the last predictin state from the stacked prediction vector}$

The system ends uop into a system of linear inequalities, which can be computationally heavy.

The first semplification is the `Ellipsoidal terminal set (Lyapunov level set)`: $\mathcal{X}_f = \{ x : x^T P x \leq \alpha \}$

where: $P$ comes from solving a Lyapunov equation for the local LQR controller.

Easy to compute since it is a single quadratic inequality constraint.

A more simplified alternative is the `Terminal equality set`: $\mathcal{X}_f = \{ x \, | \, x_{N_p} = ref \}$

In this case it is add a simple equality contraint (very conservative).


| Type                                             | Feasibility Region Size | Computational Cost       | Stability Guarantee | Common Use Case                 |
| ------------------------------------------------ | ----------------------- | ------------------------ | ------------------- | ------------------------------- |
| Maximal polyhedral invariant                     | Largest                 | High (many inequalities) | Strong              | When maximizing operating range |
| Ellipsoidal                                      | Medium                  | Low (1 quadratic)        | Strong              | Most practical linear MPC       |
| Equality ($x_{N_p}=ref$)                         | Small                   | Very low                 | Strong              | Simple systems, short horizons  |
| None (rely only on long horizon + terminal cost) | Large (with long $N_p$) | Lowest                   | Weaker/practical    | Many industrial applications    |

