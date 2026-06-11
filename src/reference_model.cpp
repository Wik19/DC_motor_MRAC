#include "reference_model.hpp"
#include "integrator.hpp"
#include <array>

void ReferenceModel::step(double r, double dt) {
    const double wn = cfg_.omega_n, z = cfg_.zeta;
    std::array<double, 2> x = {y_, ydot_};
    auto f = [&](const std::array<double, 2>& s) -> std::array<double, 2> {
        return {s[1], -wn * wn * s[0] - 2.0 * z * wn * s[1] + wn * wn * r};
    };
    x = rk4_step(x, dt, f);
    y_ = x[0];
    ydot_ = x[1];
}
