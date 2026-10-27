# Benchmark notes

## Objective

This project evaluates a hybrid controller for a tank-level process in industrial automation. The system compares a conventional PID controller with a predictive MPC controller, then combines them inside a supervisory hybrid architecture.

## Process model

The nonlinear process is described by:

A dh/dt = q_in - k_out sqrt(h)

This reflects one of the most common industrial process dynamics: a storage tank with inflow control and gravity-driven outflow.

## Control logic

The PID controller minimizes instantaneous tracking error and is effective for nominal conditions. The MPC controller minimizes a multistep cost with predicted future behavior and actuator constraints. The hybrid controller chooses MPC when the process is far from setpoint or near operational limits, and otherwise uses PID.

## Benchmark outputs

The benchmark captures:
- average control latency
- integral absolute error (IAE)
- actuator effort and smoothness

## Interpretation

A hybrid strategy is useful in resource-constrained automation because it reduces unnecessary optimization overhead while maintaining predictive performance under critical conditions.

## Application relevance

This project is directly relevant to:
- liquid storage systems
- chemical process units
- water treatment plants
- industrial surge tanks
- refinery and utility operations
