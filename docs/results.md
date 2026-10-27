# Benchmark notes

## Overview

This repository targets a classic industrial automation benchmark: controlling the level in a nonlinear tank process.

## Controlled process

The process is represented by a nonlinear first-order model:

A * dh/dt = q_in - k_out * sqrt(h)

The behavior is nonlinear because the outflow depends on the square root of the tank height.

## Controller comparison

- PID controller: simple, fast, low computational cost
- MPC controller: better predictive performance under constraint handling
- Hybrid controller: practical blend of both perspectives

## Interpretation

The hybrid controller is especially attractive in industrial systems because it combines robustness with computational practicality. PID is ideal under normal conditions, while MPC is engaged during large excursions or near-constraint operation.

## Recommended use cases

- tank-level regulation
- inflow/outflow balancing
- process plants with actuator saturation
- resource-constrained embedded control systems

## Summary

The project demonstrates a realistic industrial automation workflow, from modeling to controller implementation and benchmarking.
