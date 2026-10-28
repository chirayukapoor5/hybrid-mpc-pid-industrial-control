# Hybrid MPC–PID Tank-Level Control

> A compact industrial-process control demo combining PID feedback with a constrained, OSQP-based predictive controller.

The project models a tank-level process and compares standalone PID, standalone MPC, and hybrid PID–MPC control. The hybrid controller keeps PID feedback in the loop and adds an MPC contribution before applying actuator limits.

## At a glance

| Component | Purpose |
|---|---|
| Tank process | Nonlinear inflow/outflow model |
| PID controller | Feedback correction from setpoint error |
| MPC controller | Bounded quadratic optimization using OSQP |
| Hybrid controller | Combines PID feedback and predictive action |
| Python simulation | Writes tank data and plots to `python/` |

The tank model is:

```text
A · dh/dt = q_in - k_out · sqrt(h)
```

The combined command is bounded before it reaches the process:

```text
u_cmd = clamp(u_PID + u_MPC, u_min, u_max)
```

## Build and run

### Requirements

- CMake 3.18 or newer
- A C++17 compiler
- Python 3.10 or newer for the simulation scripts

Install the Python packages from the repository root:

```powershell
python -m pip install -r requirements.txt
```

### Build the C++ application

Run these commands from the repository root:

```powershell
cmake -S . -B build
cmake --build build --parallel
.\build\src\hybrid_control_app.exe
```

On Linux or macOS, run the built executable with:

```bash
./build/src/hybrid_control_app
```

### Run the Python simulation and plot

```powershell
python python\run_simulation.py
python python\plot_results.py
```

The simulation writes `python/tank_simulation.csv` and `python/tank_simulation.png`; the plotting script writes `python/summary_plot.png`. These are generated outputs and are excluded from Git.

## Sample benchmark results

One local run of the C++ benchmark produced the following results:

| Controller | IAE (200 samples) | Average controller time |
|---|---:|---:|
| PID | 0.521373 | 0 ms* |
| MPC | 17.8192 | 0.051315 ms |
| Hybrid PID–MPC | 1.13934 | 0 ms* |

The run used a 0.5 s sample interval and a 0.55 tank-level setpoint. IAE is the sum of absolute level errors across the 200 samples. The benchmark prints latency with limited precision, so `0 ms*` means the measured average rounded to zero in its output—not that execution takes no time.

These are illustrative measurements from one development-machine run, not guaranteed performance figures. In this run the standalone MPC had higher IAE than PID; the results should not be read as proof that one controller is universally better. Benchmark timing is not a worst-case execution-time guarantee for industrial hardware.

## Project layout

```text
python/       Tank simulation, CSV output, and plotting scripts
src/common/   Tank process model
src/pid/      PID feedback controller
src/mpc/      OSQP-based predictive controller
src/hybrid/   Hybrid PID–MPC controller
src/benchmark/Benchmark and metric reporting
docs/         Formulation and results notes
```

## Notes and documentation

The MPC implementation is a compact, one-step quadratic optimization demo; it is not a certified industrial controller. Real deployment requires validation on the target process and hardware, including safety, constraint, and worst-case timing analysis.

- [Mathematical formulation](docs/mathematical_formulation.md)
- [Results and benchmark notes](docs/results.md)
