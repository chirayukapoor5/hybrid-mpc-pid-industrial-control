# Resource-Constrained Hybrid MPC–PID Control for Industrial Process Automation

This project implements a hybrid level-control strategy for a tank process used in industrial automation. The control architecture follows the ABB-inspired pattern of combining a conventional PID feedback loop with predictive model-based feedforward action.

The controller equation is:

u_total = u_PID + u_MPC

where:
- u_PID is the feedback correction computed from the error e = r - h
- u_MPC is a predictive feedforward contribution computed from the model and current operating point
- final actuator authority is clipped to the allowed bounds

This is the architecture most directly aligned with the ABB description: an existing PID loop remains in place, while MPC contributes an anticipatory correction based on the predicted future process behavior.

## 1. Process model

The tank dynamics are modeled as:

A dh/dt = q_in - k_out sqrt(h)

with a discrete-time approximation:

h_{k+1} = h_k + (dt / A) (q_k - k_out sqrt(h_k))

The nonlinear model is used for process simulation in Python, and a local linearized model is used inside the MPC optimization.

## 2. MPC feedforward contribution

The predictive controller solves a small quadratic program around the current operating point. The formulation is based on the standard tracking objective:

min_{u_{ff}} sum_{i=1}^{N_p} Q (h_{k+i} - r)^2 + sum_{i=0}^{N_c-1} R (Delta u_{k+i})^2

subject to:
- u_min <= u_PID + u_ff <= u_max
- h_min <= h <= h_max
- input-rate limits, where used

This project uses a simplified one-step predictive feedforward implementation to keep the controller compact and easy to compile while still preserving the essential ABB-style structure.

## 3. Hybrid signal combination

The actual implemented command law is:

u_cmd = clamp(u_PID + u_MPC_ff, u_min, u_max)

with:
- u_PID = PID(r, h, dt)
- u_MPC_ff = feedforward term from the linearized predictive model
- clamp is applied to respect actuator bounds and to prevent unstable wind-up in saturated operation

The hybrid supervisory logic only enables MPC feedforward when the system is outside the nominal operating band, when the error is large, or when the process is entering a transient condition that benefits from predictive action.

## 4. Repository structure

- `python/`: process simulation and plots
- `src/common/`: tank model
- `src/pid/`: feedback controller
- `src/mpc/`: predictive controller and feedforward optimizer
- `src/hybrid/`: ABB-style hybrid supervisor
- `src/benchmark/`: benchmarking and metric collection
- `docs/`: derivations and design notes

## 5. Build and run

### Requirements
- CMake 3.16+
- C++17 compiler
- Python 3.10+
- NumPy
- Matplotlib

### Build the C++ application

```bash
mkdir -p build
cmake -S . -B build
cmake --build build
./build/hybrid_control_app
```

### Run the Python tank simulation

```bash
python3 -m pip install -r requirements.txt
python3 python/run_simulation.py
```

### Plot saved results

```bash
python3 python/plot_results.py
```

## 6. Experimental plan

The benchmark script evaluates multiple scenarios:
- setpoint step change
- inflow disturbance
- actuator saturation
- model mismatch
- solver failure fallback
- different prediction horizons

The project records:
- IAE
- overshoot and settling time
- constraint violations
- mean, 95th percentile, and max execution time
- peak memory footprint
- MPC activation ratio
- solver iteration count

Important note: laptop measurements do not establish a real-time worst-case execution-time guarantee for ABB hardware. They provide a useful development benchmark and an empirical design signal, not a certified real-time bound.

## 7. Documentation

See:
- `docs/mathematical_formulation.md`
- `docs/results.md`
