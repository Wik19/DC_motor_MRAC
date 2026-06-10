#pragma once

// One reference sample: desired motor position and its first two derivatives.
struct RefSample {
    double pos = 0.0;  // theta_md
    double vel = 0.0;  // theta_md_dot
    double acc = 0.0;  // theta_md_ddot
};

enum class RefType { Step, Square, Sine, Cubic };

struct ReferenceConfig {
    RefType type = RefType::Sine;
    double amplitude = 50.0;  // [rad] motor side (Step / Square / Sine)
    double omega = 3.0;       // [rad/s] (Sine)
    double period = 2.0;      // [s] (Square)
    double t_move = 1.0;      // [s] move duration (Cubic)
    double start = 0.0;       // [rad] start position (Cubic)
    double target = 50.0;     // [rad] end position (Cubic)
};

// Reference signal generator (manual section 2.1): step, square wave,
// sinusoid, or rest-to-rest cubic trajectory.
class ReferenceModel {
public:
    explicit ReferenceModel(const ReferenceConfig& cfg) : cfg_(cfg) {}
    RefSample eval(double t) const;

private:
    RefSample cubic(double t) const;
    ReferenceConfig cfg_;
};
