#include "trajectory.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>

TrajPoint Trajectory::eval(double t) const {
    switch (cfg_.type) {
        case TrajType::Step:
            return {cfg_.amplitude, 0.0, 0.0};
        case TrajType::Square: {
            const double phase = std::fmod(t, cfg_.period);
            const double sign = (phase < 0.5 * cfg_.period) ? 1.0 : -1.0;
            return {sign * cfg_.amplitude, 0.0, 0.0};
        }
        case TrajType::SquareVar: {
            // 1. Figure out which cycle we are on
            const auto n = static_cast<std::size_t>(t / cfg_.period);
            
            // 2. Grab the exact number from your custom list
            const double A = cfg_.amp_levels.empty()
                                 ? cfg_.amplitude
                                 : cfg_.amp_levels[n % cfg_.amp_levels.size()];
                                 
            // 3. Return the number exactly as it is (no 'sign' multiplication!)
            return {A, 0.0, 0.0};
        }
        case TrajType::Sine: {
            const double A = cfg_.amplitude, w = cfg_.omega;
            return {A * std::sin(w * t), A * w * std::cos(w * t),
                    -A * w * w * std::sin(w * t)};
        }
        case TrajType::MultiSine: {
            TrajPoint p;
            const std::size_t n =
                std::min(cfg_.sine_amps.size(), cfg_.sine_freqs.size());
            for (std::size_t k = 0; k < n; ++k) {
                const double A = cfg_.sine_amps[k], w = cfg_.sine_freqs[k];
                p.pos += A * std::sin(w * t);
                p.vel += A * w * std::cos(w * t);
                p.acc += -A * w * w * std::sin(w * t);
            }
            return p;
        }
        case TrajType::Cubic:
            return cubic(t);
    }
    return {};
}

TrajPoint Trajectory::cubic(double t) const {
    const double tf = cfg_.t_move;
    if (t >= tf) return {cfg_.target, 0.0, 0.0};

    // Eq. (31) with zero start/end velocities (rest-to-rest).
    const double d = cfg_.target - cfg_.start;
    const double a2 = 3.0 * d / (tf * tf);
    const double a3 = -2.0 * d / (tf * tf * tf);
    return {cfg_.start + a2 * t * t + a3 * t * t * t, 2.0 * a2 * t + 3.0 * a3 * t * t,
            2.0 * a2 + 6.0 * a3 * t};
}
