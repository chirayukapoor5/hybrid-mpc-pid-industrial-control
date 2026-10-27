#include "common/TankModel.hpp"

#include <algorithm>
#include <cmath>

TankModel::TankModel(double area, double qInMax, double outCoeff)
    : area_(area), qInMax_(qInMax), outCoeff_(outCoeff) {}

double TankModel::step(double h, double qIn, double dt) {
    const double hSafe = std::max(0.0, h);
    const double qOut = outCoeff_ * std::sqrt(hSafe);
    const double hDot = (qIn - qOut) / area_;
    return std::max(0.0, h + hDot * dt);
}

double TankModel::linearizedA(double h0, double dt) const {
    const double hSafe = std::max(1e-6, h0);
    return 1.0 - dt * (outCoeff_ / (2.0 * area_ * std::sqrt(hSafe)));
}

double TankModel::linearizedB(double dt) const {
    return dt / area_;
}

double TankModel::getArea() const { return area_; }
double TankModel::getQInMax() const { return qInMax_; }
double TankModel::getOutCoeff() const { return outCoeff_; }
