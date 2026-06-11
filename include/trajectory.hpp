#pragma once

#include <vector>

// One trajectory sample: desired motor position and its first two derivatives.
struct TrajPoint {
    double pos = 0.0;  // theta_md
    double vel = 0.0;  // theta_md_dot
    double acc = 0.0;  // theta_md_ddot
};

enum class TrajType { Step, Square, SquareVar, Sine, MultiSine, Cubic };

struct TrajectoryConfig {
    TrajType type = TrajType::Sine;
    double amplitude = 50.0;  // [rad] motor side (Step / Square / Sine)
    double omega = 3.0;       // [rad/s] (Sine)
    double period = 2.0;      // [s] (Square / SquareVar half-cycle period)
    double t_move = 1.0;      // [s] move duration (Cubic)
    double start = 0.0;       // [rad] start position (Cubic)
    double target = 50.0;     // [rad] end position (Cubic)

    std::vector<double> amp_levels{};

    // MultiSine: r(t) = sum_k sine_amps[k] * sin(sine_freqs[k] * t). Several
    // distinct freqs make it persistently exciting. Parallel, equal-length.
    std::vector<double> sine_amps{};   // [rad]
    std::vector<double> sine_freqs{};  // [rad/s]
};

// Command generator (manual section 2.1, "GSR"): produces the desired motion
// to track as a pure function of time. This is the command r(t), not the MRAC
// reference model.
class Trajectory {
public:
    explicit Trajectory(const TrajectoryConfig& cfg) : cfg_(cfg) {}
    TrajPoint eval(double t) const;

private:
    TrajPoint cubic(double t) const;
    TrajectoryConfig cfg_;
};
