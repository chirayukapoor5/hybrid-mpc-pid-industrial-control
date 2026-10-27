import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def plot_csv(csv_path: str, output_path: str):
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise ValueError(f"No rows found in {csv_path}")

    times = [float(r["time"]) for r in rows]
    heights = [float(r["height"]) for r in rows]
    setpoints = [float(r["setpoint"]) for r in rows]

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(times, heights, label="Tank height", color="tab:blue")
    ax.plot(times, setpoints, label="Setpoint", linestyle="--", color="tab:red")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Height")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    print(f"Saved summary plot to {output_path}")


if __name__ == "__main__":
    base_dir = os.path.dirname(os.path.abspath(__file__))
    csv_path = os.path.join(base_dir, "tank_simulation.csv")
    output = os.path.join(base_dir, "summary_plot.png")
    plot_csv(csv_path, output)
