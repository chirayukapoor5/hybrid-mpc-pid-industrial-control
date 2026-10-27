# Resource-Constrained Hybrid MPC-PID Control for Industrial Process Automation

This repository presents an end-to-end industrial process control prototype combining:
- Python-based nonlinear tank simulation
- C++ PID controller for baseline regulation
- constrained MPC implemented with a quadratic-programming solver (OSQP)
- hybrid supervisory logic that blends PID and MPC for resource-aware operation
- benchmarking and documentation

## Project objective

The goal is to regulate the level of a liquid tank while respecting actuator constraints and computational limits. The system demonstrates how a hybrid controller can balance:
- fast, simple PID response for nominal operation
- predictive MPC when the process is far from setpoint or near constraints
- lower computational burden than full MPC at all times

## Process model

The process is modeled as a nonlinear tank:

A * dh/dt = q_in - q_out

with the outflow approximated as:

q_out = k_out * sqrt(h)

This nonlinear first-order model is used in the Python simulation, while the C++ MPC uses a local linearized model around the operating point.

## Repository structure

- `python/`: simulation and plotting scripts
- `src/`: C++ implementation of the controllers and benchmark app
- `docs/`: benchmark notes and technical summary
- `CMakeLists.txt`: project build configuration

## Build and run

### Requirements
- CMake 3.16+
- C++17 compiler
- Python 3.10+
- NumPy
- Matplotlib

### C++ build

```bash
mkdir -p build
cmake -S . -B build
cmake --build build
./build/hybrid_control_app
```

### Python simulation

```bash
python3 -m pip install -r requirements.txt
python3 python/run_simulation.py
```

A plot and CSV file will be generated in the `python/` directory.

## What is included

1. Python tank simulation
   - nonlinear model
   - disturbance and setpoint handling
   - plotting

2. C++ PID controller
   - proportional-integral-derivative tuning
   - actuator bounds
   - anti-windup style integral behavior

3. Constrained MPC
   - quadratic-programming formulation using OSQP
   - bounded input constraints
   - predictive regulation around a setpoint

4. Benchmarking and documentation
   - latency comparison for PID and MPC
   - integral absolute error comparison
   - structured research summary

## Example benchmark output

```text
PID avg latency: 0.32 ms
MPC avg latency: 1.84 ms
Hybrid obj score: 93.6
PID IAE: 47.2
MPC IAE: 29.8
Hybrid IAE: 27.1
```

## Notes

This project is intentionally educational and research-oriented. It is designed to be easy to understand, compile, and extend for academic or prototype use.

