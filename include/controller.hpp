#pragma once
#include "motor.hpp"
#include "trajectory.hpp"
#include "velocity_estimator.hpp"

// Baseline (non-adaptive) controller from the manual: state feedback from the
// tracking error plus feed-forward. Used as the reference to compare MRAC
// against. It assumes the plant parameters are known exactly.

struct ControllerGains {
    double k1 = 1.0;      // position-error gain k1 (eq. 42)
    double k2 = 0.02;     // velocity-error gain k2 (eq. 42)
    double u_max = 12.0;  // control saturation [V] (eq. 44)
    double T_f = 0.025;   // velocity-estimator time constant [s] (eq. 46)
    bool feedforward = true;
};

// (eq. 41):
//   Hbar = [[0, 1], [-k_I k1/(JR), -(R b + k_I k_E + k_I k2)/(JR)]]
ControllerGains place_gains(const MotorParams& p, double omega_n, double zeta);

// Signals produced each control step
struct ControlOutput {
    double ref_pos = 0.0;
    double ref_vel = 0.0;
    double omega_est = 0.0;
    double e1 = 0.0;
    double e2 = 0.0;
    double u_ff = 0.0;
    double u_a = 0.0;
};

class Controller {
public:
    Controller(const MotorParams& p, const ControllerGains& g, double T_p)
        : p_(p), g_(g), vel_(g.T_f, T_p) {}

    ControlOutput update(double theta_m, const TrajPoint& ref);

private:
    double feedforward(const TrajPoint& ref) const;

    MotorParams p_;
    ControllerGains g_;
    VelocityEstimator vel_;
};
