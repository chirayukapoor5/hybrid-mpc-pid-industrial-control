# Resource-Constrained Hybrid MPC–PID Control for Industrial Process Automation

This project presents a hybrid control strategy for a level-control process commonly found in industrial automation systems. The goal is to regulate tank liquid height under actuator limits while managing computational effort. The solution combines a classical PID controller with constrained model predictive control (MPC), and applies a supervisory policy that chooses the appropriate controller depending on operating conditions.

## 1. Problem formulation

A tank-level process can be modeled by a nonlinear mass balance:

A dh/dt = q_in - q_out

where:
- A is tank cross-sectional area
- q_in is inflow to the tank
- q_out = k_out * sqrt(h) is the outflow depending on height

The nonlinear state dynamics are therefore:

h_dot = (q_in - k_out * sqrt(h)) / A

This model is representative of many industrial tanks, sumps, and surge vessels.

## 2. Control architecture

The controller combines two strategies:
- PID: used in nominal operation for simplicity and speed
- MPC: used when the system is far from setpoint, highly dynamic, or near constraint boundaries

A hybrid supervisor selects between them based on tracking error and operating conditions.

## 3. Mathematical modeling

A linearized discrete-time model around an operating point is used for MPC:

x[k+1] = A x[k] + B u[k]

y[k] = C x[k]

For the tank-level process:
- x = h
- u = q_in
- A = 1 - dt * (k_out / (2 A sqrt(h0)))
- B = dt / A

The predictive controller minimizes a quadratic cost over a finite horizon:

J = sum_{k=0}^{N-1} (x[k] - x_ref)^T Q (x[k] - x_ref) + u[k]^T R u[k]

with input constraints:

u_min <= u[k] <= u_max

## 4. Repository structure

- `python/`: nonlinear simulation and data plotting
- `src/common/`: tank model utilities
- `src/pid/`: PID implementation in C++
- `src/mpc/`: constrained MPC controller in C++
- `src/hybrid/`: hybrid switching logic
- `src/benchmark/`: performance comparison and benchmark calculations
- `docs/`: technical notes

## 5. Build and execution

Requirements:
- CMake 3.16+
- C++17 compiler
- Python 3.10+
- NumPy
- Matplotlib

### Build C++ project

```bash
mkdir -p build
cmake -S . -B build
cmake --build build
./build/hybrid_control_app
```

### Run Python simulation

```bash
python3 -m pip install -r requirements.txt
python3 python/run_simulation.py
```

### Generate benchmark plots

```bash
python3 python/plot_results.py
```

## 6. Expected results

The benchmark compares:
- PID-only regulation quality and latency
- MPC-only regulation quality and latency
- Hybrid PID–MPC operation under dynamic conditions

The hybrid strategy is intended to provide a practical trade-off:
- PID is low-computation and fast
- MPC improves tracking and constraint satisfaction when needed
- hybrid logic reduces computational demand while preserving performance

## 7. Notes

This is a compact but complete educational and research-grade control project suitable for demonstrating industrial process automation, model-based control, and performance benchmarking.
