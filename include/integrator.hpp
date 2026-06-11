#pragma once
#include <array>
#include <cstddef>

// One fixed-step RK4 update
template <std::size_t N, typename Deriv>
std::array<double, N> rk4_step(const std::array<double, N>& x, double dt, Deriv f) {
    std::array<double, N> tmp;
    const auto k1 = f(x);
    for (std::size_t i = 0; i < N; ++i) tmp[i] = x[i] + 0.5 * dt * k1[i];
    const auto k2 = f(tmp);
    for (std::size_t i = 0; i < N; ++i) tmp[i] = x[i] + 0.5 * dt * k2[i];
    const auto k3 = f(tmp);
    for (std::size_t i = 0; i < N; ++i) tmp[i] = x[i] + dt * k3[i];
    const auto k4 = f(tmp);

    std::array<double, N> out;
    for (std::size_t i = 0; i < N; ++i)
        out[i] = x[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    return out;
}
