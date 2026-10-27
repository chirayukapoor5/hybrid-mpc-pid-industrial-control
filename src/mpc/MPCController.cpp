#include "mpc/MPCController.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

extern "C" {
#include "osqp.h"
}

MPCController::MPCController(double dt, int horizon,
                             double qWeight, double rWeight, double deltaWeight)
    : dt_(dt), horizon_(horizon), qWeight_(qWeight), rWeight_(rWeight), deltaWeight_(deltaWeight),
      uMin_(0.0), uMax_(0.45) {}

void MPCController::configure(double uMin, double uMax) {
    uMin_ = uMin;
    uMax_ = uMax;
}

double MPCController::compute(double currentHeight, double setpoint, double previousInput) {
    // Lightweight 1D QP approximation for a tank-level predictive controller.
    // The optimization minimizes tracking error and smoothness while respecting bounds.
    const int n = 1;
    const int m = 1;

    const double trackingPenalty = qWeight_;
    const double inputPenalty = rWeight_;
    const double deltaPenalty = deltaWeight_;

    const double objectiveQuadratic = 2.0 * (trackingPenalty + inputPenalty + deltaPenalty);
    const double objectiveLinear = -2.0 * (trackingPenalty * setpoint + deltaPenalty * previousInput);

    std::vector<c_float> P_x = {static_cast<c_float>(objectiveQuadratic)};
    std::vector<c_int> P_i = {0};
    std::vector<c_int> P_p = {0, 1};

    std::vector<c_float> A_x = {1.0f};
    std::vector<c_int> A_i = {0};
    std::vector<c_int> A_p = {0, 1};

    std::vector<c_float> q = {static_cast<c_float>(objectiveLinear)};
    std::vector<c_float> l = {static_cast<c_float>(uMin_)};
    std::vector<c_float> u = {static_cast<c_float>(uMax_)};

    csc* P = csc_matrix(n, n, 1, P_x.data(), P_i.data(), P_p.data());
    csc* A = csc_matrix(n, m, 1, A_x.data(), A_i.data(), A_p.data());

    OSQPSettings* settings = osqp_settings_create();
    osqp_set_default_settings(settings);
    settings->eps_abs = 1e-5;
    settings->eps_rel = 1e-5;
    settings->max_iter = 10000;
    settings->verbose = 0;

    OSQPData* data = static_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
    data->n = n;
    data->m = m;
    data->P = P;
    data->q = q.data();
    data->A = A;
    data->l = l.data();
    data->u = u.data();

    OSQPWorkspace* work = osqp_setup(data, settings);
    double optimalInput = previousInput;

    if (work != nullptr) {
        int exitCode = osqp_solve(work);
        if (exitCode == 0 && work->solution != nullptr && work->solution->x != nullptr) {
            optimalInput = static_cast<double>(work->solution->x[0]);
        }
        osqp_cleanup(work);
    }

    osqp_settings_destroy(settings);
    c_free(data);
    c_free(P);
    c_free(A);

    return std::clamp(optimalInput, uMin_, uMax_);
}
