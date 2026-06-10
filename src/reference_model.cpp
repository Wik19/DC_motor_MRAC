#include "reference_model.hpp"
#include <cmath>

RefSample ReferenceModel::eval(double t) const {
    switch (cfg_.type) {
        case RefType::Step:
            return {cfg_.amplitude, 0.0, 0.0};
        case RefType::Square: {
            const double phase = std::fmod(t, cfg_.period);
            const double sign = (phase < 0.5 * cfg_.period) ? 1.0 : -1.0;
            return {sign * cfg_.amplitude, 0.0, 0.0};
        }
        case RefType::Sine: {
            const double A = cfg_.amplitude, w = cfg_.omega;
            return {A * std::sin(w * t), A * w * std::cos(w * t),
                    -A * w * w * std::sin(w * t)};
        }
        case RefType::Cubic:
            return cubic(t);
    }
    return {};
}

RefSample ReferenceModel::cubic(double t) const {
    const double tf = cfg_.t_move;
    if (t >= tf) return {cfg_.target, 0.0, 0.0};

    // Eq. (31) with zero start/end velocities (rest-to-rest).
    const double d = cfg_.target - cfg_.start;
    const double a2 = 3.0 * d / (tf * tf);
    const double a3 = -2.0 * d / (tf * tf * tf);
    return {cfg_.start + a2 * t * t + a3 * t * t * t, 2.0 * a2 * t + 3.0 * a3 * t * t,
            2.0 * a2 + 6.0 * a3 * t};
}
