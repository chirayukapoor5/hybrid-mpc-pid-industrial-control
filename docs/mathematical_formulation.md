# Resource-Constrained Hybrid MPC–PID Control for Industrial Process Automation

This project implements a practical hybrid control architecture for an industrial tank-level process. It follows the ABB-inspired pattern in which a classical PID controller provides the feedback correction, while a constrained MPC block contributes an anticipatory feedforward term.

The adopted control law is:

u_total = u_PID + u_MPC,ff

and the final command is then bounded as:

u_cmd = clamp(u_total, u_min, u_max)

This is not the same as switching entirely between PID and MPC. Rather, the PID controller remains active as the primary feedback loop, while the MPC generates a predictive correction term that improves tracking, disturbance rejection, and constraint handling.

## 1. Nonlinear tank model

A common tank model is given by:

A dh/dt = q_in - k_out sqrt(h)

with the discrete-time approximation:

h_{k+1} = h_k + (dt / A) (q_k - k_out sqrt(h_k))

This model is used for the Python-based simulation and for deriving the local linearized prediction model inside the MPC.

## 2. Linearized prediction model for MPC

Near the operating point h0, define:

x = h
u = q_in

The local linearized model is:

x_{k+1} = A_L x_k + B_L u_k

where:

A_L = 1 - dt * (k_out / (2 A sqrt(h0)))

B_L = dt / A

This form is then used in the QP formulation. The linearized model is only valid locally, so constraints and tuning must be selected carefully to avoid invalid predictions away from the design point.

## 3. Objective function

The predictive controller minimizes a cost of the form:

J = sum_{i=1}^{N_p} Q (h_{k+i} - r)^2 + sum_{i=0}^{N_c-1} R (Delta u_{k+i})^2

with constraints:

u_min <= u_{k+i} <= u_max

h_min <= h_{k+i} <= h_max

where:
- N_p = prediction horizon
- N_c = control horizon
- r = target level
- Q = tracking weight
- R = move-suppression weight

## 4. Exact signal combination in the code

The implemented hybrid law is:

u_cmd = clamp(u_PID + u_MPC_ff, u_min, u_max)

where:
- u_PID is the immediate feedback correction from the PID controller
- u_MPC_ff is the predictive feedforward contribution from the QP-based MPC block

This matches the ABB-style architecture more closely than a full PID/MPC switching mechanism.

## 5. Implementation details

- The PID controller is implemented in `src/pid/PIDController.*`
- The MPC feedforward solver is implemented in `src/mpc/MPCController.*`
- The hybrid controller is implemented in `src/hybrid/HybridController.*`
- The benchmark logic is implemented in `src/benchmark/Benchmark.*`

## 6. Execution and experiments

### Build

```bash
mkdir -p build
cmake -S . -B build
cmake --build build
./build/hybrid_control_app
```

### Simulation

```bash
python3 -m pip install -r requirements.txt
python3 python/run_simulation.py
```

### Experiment scenarios

The benchmark script tests:
- setpoint steps
- inflow disturbances
- actuator saturation
- model mismatch
- solver failure fallback
- different prediction horizons

### Metrics recorded

- IAE
- overshoot and settling time
- number and magnitude of constraint violations
- mean / 95th-percentile / maximum execution time
- peak memory consumption
- solver iteration count
- proportion of time MPC is active

Important: the measured execution times are benchmark values from a development machine and do not by themselves establish a hard real-time worst-case bound for ABB hardware.

## 7. Additional documentation

See:
- `docs/mathematical_formulation.md`
- `docs/results.md`
