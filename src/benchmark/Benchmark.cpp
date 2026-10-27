#include "benchmark/Benchmark.hpp"

#include "common/TankModel.hpp"
#include "hybrid/HybridController.hpp"
#include "mpc/MPCController.hpp"
#include "pid/PIDController.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

void runBenchmark() {
    const double dt = 0.5;
    const double setpoint = 0.55;
    const double initialHeight = 0.25;
    const int steps = 200;

    TankModel tank(1.5, 0.45, 0.12);
    PIDController pid(1.8, 0.75, 0.25, 0.0, 0.45);
    MPCController mpc(0.5, 5, 12.0, 0.8, 0.4);
    mpc.configure(0.0, 0.45);
    HybridController hybrid(1.8, 0.75, 0.25, 0.0, 0.45);
    hybrid.configure(setpoint);

    double hPid = initialHeight;
    double hMpc = initialHeight;
    double hHybrid = initialHeight;
    double pidInput = 0.2;
    double mpcInput = 0.2;
    double hybridInput = 0.2;

    double pidTotalError = 0.0;
    double mpcTotalError = 0.0;
    double hybridTotalError = 0.0;

    std::vector<double> pidLatencies;
    std::vector<double> mpcLatencies;
    std::vector<double> hybridLatencies;

    for (int i = 0; i < steps; ++i) {
        auto pidStart = std::chrono::high_resolution_clock::now();
        pidInput = pid.update(setpoint, hPid, dt);
        auto pidEnd = std::chrono::high_resolution_clock::now();
        pidLatencies.push_back(std::chrono::duration<double, std::milli>(pidEnd - pidStart).count());

        auto mpcStart = std::chrono::high_resolution_clock::now();
        mpcInput = mpc.compute(hMpc, setpoint, mpcInput);
        auto mpcEnd = std::chrono::high_resolution_clock::now();
        mpcLatencies.push_back(std::chrono::duration<double, std::milli>(mpcEnd - mpcStart).count());

        auto hybridStart = std::chrono::high_resolution_clock::now();
        hybridInput = hybrid.update(hHybrid, dt);
        auto hybridEnd = std::chrono::high_resolution_clock::now();
        hybridLatencies.push_back(std::chrono::duration<double, std::milli>(hybridEnd - hybridStart).count());

        hPid = tank.step(hPid, pidInput, dt);
        hMpc = tank.step(hMpc, mpcInput, dt);
        hHybrid = tank.step(hHybrid, hybridInput, dt);

        pidTotalError += std::abs(setpoint - hPid);
        mpcTotalError += std::abs(setpoint - hMpc);
        hybridTotalError += std::abs(setpoint - hHybrid);
    }

    auto avgLatency = [](const std::vector<double>& vals) {
        double sum = 0.0;
        for (double v : vals) sum += v;
        return sum / static_cast<double>(vals.size());
    };

    const double pidAvg = avgLatency(pidLatencies);
    const double mpcAvg = avgLatency(mpcLatencies);
    const double hybridAvg = avgLatency(hybridLatencies);

    std::cout << "PID avg latency: " << pidAvg << " ms\n";
    std::cout << "MPC avg latency: " << mpcAvg << " ms\n";
    std::cout << "Hybrid avg latency: " << hybridAvg << " ms\n";
    std::cout << "PID IAE: " << pidTotalError << "\n";
    std::cout << "MPC IAE: " << mpcTotalError << "\n";
    std::cout << "Hybrid IAE: " << hybridTotalError << "\n";
}
