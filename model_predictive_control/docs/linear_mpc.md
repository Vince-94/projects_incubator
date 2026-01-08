# Linear MPC

- [Linear MPC](#linear-mpc)
  - [Flow](#flow)
  - [State space equation](#state-space-equation)
  - [State space equation discretized](#state-space-equation-discretized)
  - [Prediction](#prediction)
    - [State prediction](#state-prediction)
    - [Output prediction](#output-prediction)
  - [Optimization Problem (input decision variable)](#optimization-problem-input-decision-variable)
    - [Cost function](#cost-function)
      - [State](#state)
      - [Output](#output)
    - [Constraints](#constraints)
      - [State constraints](#state-constraints)
      - [Output constraints](#output-constraints)
      - [Input constraints](#input-constraints)
      - [Input rate constraints](#input-rate-constraints)
      - [Inequality constraints (state)](#inequality-constraints-state)
  - [Terminal cost and terminal state constraint](#terminal-cost-and-terminal-state-constraint)
    - [Cost function](#cost-function-1)
    - [Terminal state constraint (invariant terminal set)](#terminal-state-constraint-invariant-terminal-set)


## Flow
At k-th sampling time:
1. Predict the system future state over the prediction horizon
2. Solve QP (Quadratic Program) optimization problem
3. Check feasibility, stability, robustness, performance and real-time
4. Receiding horizon
5. Wait the k+1 sampling time and repeat from point 1


## State space equation
$$
\begin{aligned}
    \dot{x}(t) &= A\,x(t) + B\,u(t) \\
    y(t) &= C\,x(t) + D\,u(t)
\end{aligned}
$$
where:
- $A \in \mathbb{R}^{n_x, n_x} \quad \text{state matrix}$
- $B \in \mathbb{R}^{n_x, n_u} \quad \text{state-input matrix}$
- $C \in \mathbb{R}^{n_y, n_x} \quad \text{output matrix}$
- $D \in \mathbb{R}^{n_y, n_u} \quad \text{output-input matrix}$


## State space equation discretized
$$
\begin{aligned}
    x(k+1) &= A_d \, x(k) + B_d \, u(k) \rightarrow x_{k+1} = A_d \, x_k + B_d  \,u_k \\
    y(k) &= C_d \, x(k) + D_d \, u(k) \rightarrow y_k = C_d \, x_k + D_d \, u_k
\end{aligned}
$$

## Prediction
At k-th sampled, the predicted state is:
$$
\begin{aligned}
    x_{i|k+1} &= A_d \, x_{i|k} + B_d \, u_{i|k}, \quad x_{0|k} = x(0|k) \\
    y_{i|k} &= C \, x_{i|k} + D \, u_{i|k}
\end{aligned}
$$

### State prediction
$$
X_k = \begin{bmatrix} x_{1|k} \\ \vdots \\ x_{N_p|k} \end{bmatrix}
     = \underbrace{\begin{bmatrix} A_d \\ A_d^2 \\ \vdots \\ A_d^{N_p} \end{bmatrix}}_{\bar{A}} x_k
     + \underbrace{\begin{bmatrix} B_d & 0 & \cdots & 0 \\
                                  A_d B_d & B_d & \cdots & 0 \\
                                  \vdots & \vdots & \ddots & \vdots \\
                                  A_d^{N_p-1} B_d & A_d^{N_p-2} B_d & \cdots & B_d \end{bmatrix}}_{\bar{B}} U_k \\
X_k = \bar{A} \, x_k + \bar{B} \, U_k
$$

### Output prediction
$$
Y_k = \begin{bmatrix} y_{1|k} \\ \vdots \\ y_{N_p|k} \end{bmatrix}
    = \underbrace{\begin{bmatrix} C A_d \\ C A_d^2 \\ \vdots \\ C A_d^{N_p} \end{bmatrix}}_{\bar{C}} x_k
    + \underbrace{\begin{bmatrix} C B_d + D & 0 & \cdots & 0 \\
                                  C A_d B_d & C B_d + D & \cdots & 0 \\
                                  \vdots & \vdots & \ddots & \vdots \\
                                  C A_d^{N_p-1} B_d & C A_d^{N_p-2} B_d & \cdots & C B_d + D \end{bmatrix}}_{\bar{D}} U_k \\
Y_k = \bar{C} \, x_k + \bar{D} \, U_k
$$

where:
- $X_k \in \mathbb{R}^{n_x N_p} \\$
- $U_k = \begin{bmatrix} u_{0|k}^\top & u_{1|k}^\top & \cdots & u_{N_c-1|k}^\top \end{bmatrix}^\top \in \mathbb{R}^{n_u N_c} \\$
- $\bar{A} \in \mathbb{R}^{n_x N_p, n_x} \\$
- $\bar{B} \in \mathbb{R}^{n_x N_p, n_u N_c} \\$
- $\bar{C} \in \mathbb{R}^{n_y N_p, n_x} \\$
- $\bar{D} \in \mathbb{R}^{n_y N_p, n_u N_c} \\$

Define the stacked input increments:
$$
\Delta U_k = \begin{bmatrix} \Delta u_{0|k}^\top & \Delta u_{1|k}^\top & \cdots & \Delta u_{N_c-1|k}^\top \end{bmatrix}^\top \in \mathbb{R}^{n_u N_c}
$$

where:
- $\Delta u_{0|k} = u_{0|k} - u_{k-1}$
- $\Delta u_{i|k} = u_{i|k} - u_{i-1|k} \quad \forall i \geq 1$

Define $\Delta U_k$ as an affine function of $U_k$:
$$
\Delta U_k = M \, U_k - U_{k-1}
$$

where:
- $U_{k-1} = [u_{k-1} \ 0 \ \cdots \ 0]^\top \in \mathbb{R}^{n_u N_c}$
- $M = \begin{bmatrix}
    1 & 0 & 0 & \cdots & 0 & 0 \\
    -1 & 1 & 0 & \cdots & 0 & 0 \\
    0 & -1 & 1 & \cdots & 0 & 0 \\
    \vdots & \vdots & \vdots & \ddots & \vdots & \vdots \\
    0 & 0 & 0 & \cdots & 1 & 0 \\
    0 & 0 & 0 & \cdots & -1 & 1
    \end{bmatrix} \in \mathbb{R}^{n_u N_c, n_u N_c}$


## Optimization Problem (input decision variable)

QP Fomulation:
$$
\begin{aligned}
    &\min_{U_k} && \frac{1}{2} \, U_k^\top H U_k + f^\top(k) U_k \\
    &\text{s.t.} && G_{\text{ineq}} U_k \leq E_{\text{ineq}}(k) \\
\end{aligned}
$$


### Cost function

#### State
$$\begin{aligned}
    J(x_k) &= \sum_{i=1}^{N_p} \| x_{i|k} - r_{i|k} \|_Q^2 + \sum_{i=0}^{N_c-1} \| u_{i|k} \|_R^2 + \sum_{i=0}^{N_c-1} \| \Delta u_{i|k} \|_S^2 \\
    &= \| X_k - REF_k \|_{\bar{Q}}^2 + \| U_k \|_{\bar{R}}^2 + \| \Delta U_k \|_{\bar{S}}^2 \\
    &= \| \bar{A} x_k + \bar{B} U_k - REF_k \|_{\bar{Q}}^2 + \| U_k \|_{\bar{R}}^2 + \| M U_k - U_{k-1} \|_{\bar{S}}^2 \\
    &= U_k^\top \underbrace{(\bar{B}^\top \bar{Q} \bar{B} + \bar{R} + M^\top \bar{S} M)}_{H} U_k + 2 \underbrace{\left[ (\bar{A} x_k - REF_k)^\top \bar{Q} \bar{B} - U_{k-1}^\top \bar{S} M \right]}_{f(k)^\top} U_k + g(k) \\
\end{aligned}$$

where:
- $\| x_{i|k} - r_{i|k} \|_Q^2 \qquad \text{state error cost}$
- $\| u_{i|k} \|_R^2 \qquad \text{input cost}$
- $REF_k = \begin{bmatrix} ref_{1|k} & \cdots & ref_{N_p|k} \end{bmatrix}^\top \in \mathbb{R}^{n_x N_p}$
- $Q \in \mathbb{R}^{n_x, n_x} \qquad \text{state error diagonal weight matrix}$
- $R \in \mathbb{R}^{n_u, n_u} \qquad \text{input diagonal weight matrix}$
- $S \in \mathbb{R}^{n_u, n_u} \qquad \text{input diagonal weight matrix}$
- $\bar{Q} = \begin{bmatrix}
    Q & 0 & \cdots & 0 \\
    0 & Q & \cdots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    0 & 0 & \cdots & Q
    \end{bmatrix} \in \mathbb{R}^{n_x N_p, n_x N_p}, \quad Q \succeq 0$
- $\bar{R} = \begin{bmatrix}
    R & 0 & \cdots & 0 \\
    0 & R & \cdots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    0 & 0 & \cdots & R
    \end{bmatrix} \in \mathbb{R}^{n_u N_c, n_u N_c}, \quad R \succeq 0$
- $\bar{S} = \begin{bmatrix}
        S & 0 & \cdots & 0 \\
        0 & S & \cdots & 0 \\
        \vdots & \vdots & \ddots & \vdots \\
        0 & 0 & \cdots & S
    \end{bmatrix} \in \mathbb{R}^{n_u N_c, n_u N_c}, \quad S \succeq 0$

#### Output
$$
\begin{aligned}
    J(y_k) &= \sum_{i=1}^{N_p} \| y_{i|k} - r_{i|k} \|_Q^2 + \sum_{i=0}^{N_c-1} \| u_{i|k} \|_R^2 + \sum_{i=0}^{N_c-1} \| \Delta u_{i|k} \|_S^2 \\
    &= \| Y_k - REF_k \|_{\bar{Q}}^2 + \| U_k \|_{\bar{R}}^2 + \| \Delta U_k \|_{\bar{S}}^2 \\
    &= \| \bar{C} x_k + \bar{D} U_k - REF_k \|_{\bar{Q}}^2 + \| U_k \|_{\bar{R}}^2 + \| M U_k - U_{k-1} \|_{\bar{S}}^2 \\
    &= U_k^\top \underbrace{(\bar{D}^\top \bar{Q} \bar{D} + \bar{R} + M^\top \bar{S} M)}_{H} U_k + 2 \underbrace{\left[ (\bar{C} x_k - REF_k)^\top \bar{Q} \bar{D} - U_{k-1}^\top \bar{S} M \right]}_{f(k)^\top} U_k + g(k)
\end{aligned}
$$


where:
- $\| y_{i|k} - r_{i|k} \|_Q^2 \qquad \text{output error cost}$
- $\| u_{i|k} \|_R^2 \qquad \text{input cost}$
- $REF_k = \begin{bmatrix} ref_{1|k} & \cdots & ref_{N_p|k} \end{bmatrix}^\top \in \mathbb{R}^{n_y N_p}$
- $Q \in \mathbb{R}^{n_y, n_y} \qquad \text{state error diagonal weight matrix}$
- $R \in \mathbb{R}^{n_u, n_u} \qquad \text{input diagonal weight matrix}$
- $S \in \mathbb{R}^{n_u, n_u} \qquad \text{input diagonal weight matrix}$
- $\bar{Q} = \begin{bmatrix}
    Q & 0 & \cdots & 0 \\
    0 & Q & \cdots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    0 & 0 & \cdots & Q
    \end{bmatrix} \in \mathbb{R}^{n_y N_p, n_y N_p}, \quad Q \succeq 0$
- $\bar{R} = \begin{bmatrix}
    R & 0 & \cdots & 0 \\
    0 & R & \cdots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    0 & 0 & \cdots & R
    \end{bmatrix} \in \mathbb{R}^{n_u N_c, n_u N_c}, \quad R \succeq 0$
- $\bar{S} = \begin{bmatrix}
        S & 0 & \cdots & 0 \\
        0 & S & \cdots & 0 \\
        \vdots & \vdots & \ddots & \vdots \\
        0 & 0 & \cdots & S
    \end{bmatrix} \in \mathbb{R}^{n_u N_c, n_u N_c}, \quad S \succeq 0$


### Constraints

#### State constraints
$$
x_{\min} \leq x_{i|k} \leq x_{\max} \quad \rightarrow \quad
X_{\min} \leq X_k \leq X_{\max} \quad \Rightarrow \quad
\begin{cases}
    - \bar{B} \, U_k \leq - X_{\min} + \bar{A} \, x_k \\
    \bar{B} \, U_k \leq X_{\max} - \bar{A} \, x_k
\end{cases}
$$

where:
- $X_{\min} = [x_{\min,0} \ \cdots \ x_{\min,n}]^\top \in \mathbb{R}^{n_x N_p} \\$
- $X_{\max} = [x_{\max,0} \ \cdots \ x_{\max,n}]^\top \in \mathbb{R}^{n_x N_p} \\$

#### Output constraints
$$
y_{\min} \leq y_{i|k} \leq y_{\max} \quad \rightarrow \quad
Y_{\min} \leq Y_k \leq Y_{\max} \quad \Rightarrow \quad
\begin{cases}
    -\bar{D} \, U_k \leq -Y_{\min} + \bar{C} \, x_k \\
    \bar{D} \, U_k \leq Y_{\max} - \bar{C} \, x_k
\end{cases}
$$

where:
- $Y_{\min} = [y_{\min,0} \ \cdots \ y_{\min,n}]^\top \in \mathbb{R}^{n_y N_p} \\$
- $Y_{\max} = [y_{\max,0} \ \cdots \ y_{\max,n}]^\top \in \mathbb{R}^{n_y N_p} \\$

#### Input constraints
$$
u_{\min} \leq u_{i|k} \leq u_{\max} \quad \rightarrow \quad
U_{\min} \leq U_k \leq U_{\max} \quad \Rightarrow \quad
\begin{cases}
    - U_k \leq - U_{\min} \\
    U_k \leq U_{\max}
\end{cases}
$$

where:
- $U_{\min} = [u_{\min,0} \ \cdots \ u_{\min,n}]^\top \in \mathbb{R}^{n_u N_c} \\$
- $U_{\max} = [u_{\max,0} \ \cdots \ u_{\max,n}]^\top \in \mathbb{R}^{n_u N_c} \\$

#### Input rate constraints

$$
\Delta u_{\min} \leq \Delta u_{i|k} \leq \Delta u_{\max} \quad \rightarrow \quad
\Delta U_{\min} \leq \Delta U_k \leq \Delta U_{\max} \quad \Rightarrow \quad
\begin{cases}
    - M \, U_k \leq - \Delta U_{\min} - U_{k-1} \\
    M \, U_k \leq \Delta U_{\max} + U_{k-1}
\end{cases}
$$

where:
- $\Delta U_{\min} = [\Delta u_{\min,0} \ \cdots \ \Delta u_{\min,n}]^\top \in \mathbb{R}^{n_u N_c}$
- $\Delta U_{\max} = [\Delta u_{\max,0} \ \cdots \ \Delta u_{\max,n}]^\top \in \mathbb{R}^{n_u N_c}$
- $\Delta u_{i|k} = u_{i|k} - u_{i-1|k}, \qquad \Delta u_{0|k} = u_{0|k} - u_{k-1}$

#### Inequality constraints (state)
$$
\begin{cases}
    -\bar{B} \, U_k & \leq -X_{\min} + \bar{A} \, x_k \\
    \bar{B} \, U_k & \leq X_{\max} - \bar{A} \, x_k \\
    -U_k & \leq U_{\min} \\
    U_k & \leq U_{\max} \\
    -M \, U_k & \leq -\Delta U_{\min} - U_{k-1} \\
    M \, U_k & \leq \Delta U_{\max} + U_{k-1} \\
\end{cases}
\quad \Rightarrow \quad
\underbrace{\begin{bmatrix}
    -\bar{B} \\
    \bar{B} \\
    -I_{n_u N_c} \\
    I_{n_u N_c} \\
    -M \\
    M
\end{bmatrix}}_{G_{\text{ineq}}}
U_k \leq
\underbrace{\begin{bmatrix}
    -X_{\min} + \bar{A} \, x_k \\
    X_{\max} - \bar{A} \, x_k \\
    -U_{\min} \\
    U_{\max} \\
    -\Delta U_{\min} - U_{k-1} \\
    \Delta U_{\max} + U_{k-1}
\end{bmatrix}}_{E_{\text{ineq}}(k)}
$$


## Terminal cost and terminal state constraint

### Cost function

State:

$$
\begin{aligned}
    J(x_k) &= \| x_{N_p|k} - r_{N_p|k} \|_P^2 + \sum_{i=1}^{N_p-1} \| x_{i|k} - r_{i|k} \|_Q^2 + \sum_{i=0}^{N_c-1} \| u_{i|k} \|_R^2 + \sum_{i=0}^{N_c-1} \| \Delta u_{i|k} \|_S^2 \\
    &= \| \bar{A} x_k + \bar{B} U_k - REF_k \|_{\bar{Q}_f}^2 + \| U_k \|_{\bar{R}}^2 + \| M U_k - U_{k-1} \|_{\bar{S}}^2 \\
    &= U_k^\top \underbrace{(\bar{B}^\top \bar{Q}_f \bar{B} + \bar{R} + M^\top \bar{S} M)}_{H} U_k + 2 \underbrace{\left[ (\bar{A} x_k - REF_k)^\top \bar{Q}_f \bar{B} - U_{k-1}^\top \bar{S} M \right]}_{f(k)^\top} U_k + g(k)
\end{aligned}
$$

where:
- $\| x_{N_p|k} - r_{N_p|k} \|_P^2 \qquad \text{terminal cost}$
- $P⪰0 \in \mathbb{R}^{n_x, n_x} \qquad \text{terminal diagonal weight matrix, solution of the DARE for the infinite-horizon LQR}$
- $\bar{Q}_f = \begin{bmatrix}
    Q & 0 & \cdots & 0 \\
    0 & Q & \cdots & 0 \\
    \vdots & \vdots & \ddots & \vdots \\
    0 & 0 & \cdots & P
    \end{bmatrix} \in \mathbb{R}^{n_x N_p, n_x N_p}$

Output:

$$
\begin{aligned}
    J(y_k) &= \| x_{N_p|k} - x_s \|_P^2 + \sum_{i=1}^{N_p} \| y_{i|k} - r_{i|k} \|_Q^2 + \sum_{i=0}^{N_c-1} \| u_{i|k} \|_R^2 + \sum_{i=0}^{N_c-1} \| \Delta u_{i|k} \|_S^2 \\
    &= (A_{N_p} x_k + B_{N_p} U_k - x_s)^\top P (A_{N_p} x_k + B_{N_p} U_k - x_s) + \| \bar{C} x_k + \bar{D} U_k - REF_k \|_{\bar{Q}}^2 + \| U_k \|_{\bar{R}}^2 + \| M U_k - U_{k-1} \|_{\bar{S}}^2 \\
    &= U_k^\top \underbrace{(\bar{D}^\top \bar{Q} \bar{D} + \bar{R} + B_{N_p}^\top P B_{N_p} + M^\top \bar{S} M)}_{H} U_k + 2 \underbrace{\left[ (\bar{C} x_k - REF_k)^\top \bar{Q} \bar{D} + (A_{N_p} x_k - x_s)^\top P B_{N_p} - U_{k-1}^\top \bar{S} M \right]}_{f(k)^\top} U_k + g(k)
\end{aligned}
$$

where:
- $\| y_{N_p|k} - r_{N_p|k} \|_P^2 \qquad \text{terminal cost}$
- $P⪰0 \in \mathbb{R}^{n_y, n_y} \qquad \text{terminal diagonal weight matrix, solution of the DARE for the infinite-horizon LQR}$
- $A_{N_p}$, $B_{N_p} \qquad \text{are the last rows of the state prediction matrices}$
- $x_s$ is the steady-state state corresponding to the (possibly constant) output reference $r$ (solved via the steady-state target problem).

The solution of the DARE can give:
- If $P = 0$, you recover the original finite-horizon cost.
- If $P \gg Q$, the controller aggressively drives the state toward the reference at the end of the horizon.



### Terminal state constraint (invariant terminal set)
To enforce $x_{N_p|k} \in \mathcal{X}_f$, add the ellipsoidal terminal set:
$$
\mathcal{X}_f = \{ x : \| x_{N_p|k} - r_{N_p|k} \|_P^2 = (x - r_{N_p})^\top \, P \, (x - r_{N_p}) \leq \alpha \}
$$

where:
- $\alpha$ is a scaling factor chosen to ensure invariance and constraint satisfaction within $\mathcal{X}_f$. Often $\alpha = 1$ for simplicity, but it can be tuned via set scaling algorithms.
- $x_{N_p|k} = A_{N_p} x_k + B_{N_p} U_k$
- $A_{N_p}$ = last $n_x$ rows of $\bar{A}$
- $B_{N_p}$ = last $n_x$ rows of $\bar{B}$

$$
(x - r_{N_p})^\top \, P \, (x - r_{N_p}) \leq \alpha \quad \rightarrow \quad
U_k^\top \, (B_{N_p}^\top \, P \, B_{N_p}) \, U_k + 2 \, (A_{N_p} \, x_k - r_{N_p})^\top \, P \, B_{N_p} \, U_k + (A_{N_p} \, x_k - r_{N_p})^\top \, P \, (A_{N_p} \, x_k - r_{N_p}) \leq \alpha \\
\Rightarrow \quad U_k^\top \, (B_{N_p}^\top \, P \, B_{N_p}) \, U_k + [2 \, (A_{N_p} \, x_k - r_{N_p})^\top \, P \, B_{N_p}] \, U_k \leq \alpha - (A_{N_p} \, x_k - r_{N_p})^\top \, P \, (A_{N_p} \, x_k - r_{N_p})
$$

which is a quadratic inequality constraint, turning the problem into a QCQP (quadratically constrained quadratic program).

Note: In output-based formulation, since outputs are linear combinations of states ($y = C x + D u$), the terminal set is often still defined on the underlying state (even in output-based cost), because invariance requires state-level feedback.
