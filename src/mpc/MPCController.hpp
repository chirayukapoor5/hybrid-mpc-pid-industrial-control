#pragma once

class MPCController {
public:
    MPCController(double dt = 0.5,
                  int horizon = 5,
                  double qWeight = 12.0,
                  double rWeight = 0.8,
                  double deltaWeight = 0.3);

    void configure(double uMin, double uMax);
    double compute(double currentHeight,
                   double setpoint,
                   double previousInput);

private:
    double dt_;
    int horizon_;
    double qWeight_;
    double rWeight_;
    double deltaWeight_;
    double uMin_;
    double uMax_;
};
