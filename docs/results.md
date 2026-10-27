# Mathematical formulation and derivation

## 1. Nonlinear process model

The tank-level process is modeled as:

A dh/dt = q_in - k_out sqrt(h)

where:
- A is tank cross-sectional area [m^2]
- h(t) is the liquid level [m]
- q_in(t) is the inlet flow [m^3/s]
- k_out is the outlet-flow coefficient [m^(5/2)/s]

For a discrete-time implementation with sampling time dt, the nonlinear state update is:

h_{k+1} = h_k + (dt / A) (q_k - k_out sqrt(h_k))

This model is used directly in the Python simulation and to define the operating point used by the linearized MPC.

## 2. Operating point and linearization

Let the nominal operating point be (h0, q0). We define the deviation variables:

x_k = h_k - h0
u_k = q_k - q0

The nonlinear model is linearized around h0. For the single-state tank model, the local linear approximation is:

x_{k+1} = A_L x_k + B_L u_k

with:

A_L = 1 - dt * (k_out / (2 A sqrt(h0)))

B_L = dt / A

The term A_L is obtained from the derivative of the outflow term with respect to h:

d/dh (k_out sqrt(h)) = k_out / (2 sqrt(h))

This yields a locally valid first-order prediction model.

## 3. Prediction model and cost function

For prediction horizon N_p and control horizon N_c, the objective is:

J = sum_{i=1}^{N_p} Q (h_{k+i} - r)^2 + sum_{i=0}^{N_c-1} R (Delta u_{k+i})^2

where:
- r is the target setpoint
- Q is the tracking weight
- R is the input-rate weight
- Delta u_{k+i} = u_{k+i} - u_{k+i-1}

The optimization magnitude is reduced to a scalar feedforward contribution in the implementation, but the mathematical structure is the same: a quadratic objective plus actuator and state constraints.

## 4. Constraints

The controller must respect:

u_min <= u_k <= u_max

h_min <= h_k <= h_max

The flow constraint reflects pump and valve limits. The tank-height bound reflects physical and safety limits.

For the implemented ABB-style architecture, the actual command is:

u_cmd = u_PID + u_MPC,ff

and the final command is clipped to:

u_min <= u_cmd <= u_max

This ensures the combined signal respects actuator limits even when the predictive contribution and feedback correction act together.

## 5. Conversion to a QP

The standard compact QP form is:

min_{z} 1/2 z^T P z + q^T z

subject to:

l <= A z <= u

For a scalar feedforward variable z = u_MPC,ff, the OSQP problem is formed as:

P = 2 (B^2 Q + R)

q = 2 B Q (A_L h_k + B u_PID - r)

with constraints:

u_min - u_PID <= z <= u_max - u_PID

This is the exact structure used in the simplified feedforward optimization implemented in the code. It approximates the full horizon formulation while retaining the essential predictive effect of MPC.

## 6. Units and assumptions

Assumptions:
- single-state tank approximation
- diagonal (scalar) control input
- no significant transport delay
- actuator dynamics are slower than the chosen sampling time and are approximated as instantaneous within the optimization

Units:
- h [m]
- q_in [m^3/s]
- A [m^2]
- dt [s]
- k_out is chosen to match the same volumetric units as the outlet flow

## 7. Why this is valid for the project

This formulation is sufficient for a research and demonstration project because it preserves the main design ideas:
- model-based prediction
- quadratic optimization
- predictive feedforward action
- PID as a stabilizing feedback inner loop
- constraint respect through bounded control signal

The implementation is intentionally simplified compared to a production ABB controller, but it captures the important structure of the architecture and the mathematical reasoning behind it.

## 8. Practical note on real-time deployment

The benchmark execution times reported in the project are laptop measurements only. They do not constitute a formal worst-case execution-time proof for ABB hardware. A real-time deployment assessment must include:
- worst-case timing analysis
- execution on the target platform
- jitter and deadline evaluation
- scheduler analysis
- measured solver timeout handling and fallback logic
