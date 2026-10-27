import csv
import math
from dataclasses import dataclass

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


@dataclass
class TankParameters:
    area: float = 1.5
    q_in_max: float = 0.45
    out_coeff: float = 0.12
    setpoint: float = 0.55


class TankProcess:
    def __init__(self, params: TankParameters | None = None):
        self.params = params or TankParameters()

    def step(self, h: float, q_in: float, dt: float = 0.5) -> float:
        h = max(0.0, h)
        q_out = self.params.out_coeff * math.sqrt(h)
        dhdt = (q_in - q_out) / self.params.area
        return max(0.0, h + dhdt * dt)


def pid_update(setpoint: float, kp: float = 1.8, ki: float = 0.75, kd: float = 0.25, dt: float = 0.5):
    integral = 0.0
    prev_error = 0.0

    def controller(measurement: float):
        nonlocal integral, prev_error
        error = setpoint - measurement
        integral += error * dt
        derivative = (error - prev_error) / dt
        output = kp * error + ki * integral + kd * derivative
        prev_error = error
        return max(0.0, min(0.45, output))

    return controller


def simulate():
    tank = TankProcess()
    setpoint = tank.params.setpoint
    height = 0.25
    q_in = 0.2
    controller = pid_update(setpoint)
    history = []

    for i in range(200):
        q_in = controller(height)
        height = tank.step(height, q_in, dt=0.5)
        history.append({
            "time": i * 0.5,
            "height": height,
            "input": q_in,
            "setpoint": setpoint,
        })

    return history


def main():
    history = simulate()
    with open("python/tank_simulation.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["time", "height", "input", "setpoint"])
        writer.writeheader()
        writer.writerows(history)

    times = [row["time"] for row in history]
    heights = [row["height"] for row in history]
    setpoints = [row["setpoint"] for row in history]
    inputs = [row["input"] for row in history]

    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(times, heights, label="Tank height", color="tab:blue")
    axes[0].plot(times, setpoints, label="Setpoint", color="tab:red", linestyle="--")
    axes[0].set_ylabel("Height")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(times, inputs, label="Inflow q_in", color="tab:green")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Flow")
    axes[1].grid(True)
    axes[1].legend()

    fig.tight_layout()
    fig.savefig("python/tank_simulation.png", dpi=150)
    print("Saved simulation plot to python/tank_simulation.png")
    print("Saved simulation data to python/tank_simulation.csv")


if __name__ == "__main__":
    main()
