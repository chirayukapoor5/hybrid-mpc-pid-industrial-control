
#include "mpc/MPCController.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

extern "C" {
#include "osqp.h"
}

MPCController::MPCController(
    double dt,
    int horizon,
    double qWeight,
    double rWeight,
    double deltaWeight
)
    : dt_(dt),
      horizon_(horizon),
      qWeight_(qWeight),
      rWeight_(rWeight),
      deltaWeight_(deltaWeight),
      uMin_(0.0),
      uMax_(0.45) {}

void MPCController::configure(double uMin, double uMax) {
    uMin_ = uMin;
    uMax_ = uMax;
}

double MPCController::compute(
    double currentHeight,
    double setpoint,
    double previousInput
) {
    // Validate controller settings.
    if (horizon_ <= 0 ||
        dt_ <= 0.0 ||
        qWeight_ < 0.0 ||
        rWeight_ < 0.0 ||
        deltaWeight_ < 0.0 ||
        uMin_ > uMax_) {
        return std::clamp(previousInput, uMin_, uMax_);
    }

    // Single-variable quadratic optimization.
    // Note: this is a simplified control approximation,
    // not a full multi-step predictive tank model.
    constexpr OSQPInt n = 1;
    constexpr OSQPInt m = 1;

    const double trackingPenalty = qWeight_;
    const double inputPenalty = rWeight_;
    const double deltaPenalty = deltaWeight_;

    const double Pvalue =
        2.0 * (trackingPenalty + inputPenalty + deltaPenalty);

    const double qvalue =
        -2.0 * (trackingPenalty * (setpoint - currentHeight)
                + deltaPenalty * previousInput);

    // OSQP requires a positive-semidefinite P matrix.
    // Add a small regularization term if all weights are zero.
    const OSQPFloat P_x[] = {
        static_cast<OSQPFloat>(std::max(Pvalue, 1e-8))
    };
    OSQPInt P_i[] = {0};
    OSQPInt P_p[] = {0, 1};

    const OSQPFloat A_x[] = {1.0};
    OSQPInt A_i[] = {0};
    OSQPInt A_p[] = {0, 1};

    const OSQPFloat q[] = {
        static_cast<OSQPFloat>(qvalue)
    };
    const OSQPFloat l[] = {
        static_cast<OSQPFloat>(uMin_)
    };
    const OSQPFloat u[] = {
        static_cast<OSQPFloat>(uMax_)
    };

    OSQPCscMatrix* P =
        OSQPCscMatrix_new(n, n, 1, const_cast<OSQPFloat*>(P_x),
                          P_i, P_p);

    OSQPCscMatrix* A =
        OSQPCscMatrix_new(m, n, 1, const_cast<OSQPFloat*>(A_x),
                          A_i, A_p);

    if (P == nullptr || A == nullptr) {
        if (P != nullptr) OSQPCscMatrix_free(P);
        if (A != nullptr) OSQPCscMatrix_free(A);
        return std::clamp(previousInput, uMin_, uMax_);
    }

    OSQPSettings* settings = OSQPSettings_new();

    if (settings == nullptr) {
        OSQPCscMatrix_free(P);
        OSQPCscMatrix_free(A);
        return std::clamp(previousInput, uMin_, uMax_);
    }

    settings->eps_abs = 1e-5;
    settings->eps_rel = 1e-5;
    settings->max_iter = 10000;
    settings->verbose = 0;

    OSQPSolver* solver = nullptr;

    const OSQPInt setupResult =
        osqp_setup(&solver, P, q, A, l, u, m, n, settings);

    double optimalInput = std::clamp(previousInput, uMin_, uMax_);

    if (setupResult == 0 && solver != nullptr) {
        const OSQPInt solveResult = osqp_solve(solver);

        if (solveResult == 0 &&
            solver->solution != nullptr &&
            solver->solution->x != nullptr) {
            optimalInput =
                static_cast<double>(solver->solution->x[0]);
        }
    }

    if (solver != nullptr) {
        osqp_cleanup(solver);
    }

    OSQPCscMatrix_free(P);
    OSQPCscMatrix_free(A);
    OSQPSettings_free(settings);

    return std::clamp(optimalInput, uMin_, uMax_);
}
