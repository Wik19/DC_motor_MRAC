#pragma once
#include "motor.hpp"
#include "reference_model.hpp"

struct ControllerGains {
    double k1 = 1.0;      // position-error gain
    double k2 = 0.02;     // velocity-error gain
    double u_max = 12.0;  // control saturation [V]
    double T_f = 0.025;   // velocity-estimator time constant [s]
    bool feedforward = true;
};

// Pole placement: choose k1, k2 so the tracking-error dynamics (eq. 41) form a
// 2nd-order system with the given natural frequency and damping.
ControllerGains place_gains(const MotorParams& p, double omega_n, double zeta);

// Signals produced each control step (logged by the simulation).
struct ControlOutput {
    double ref_pos = 0.0;
    double ref_vel = 0.0;
    double omega_est = 0.0;
    double e1 = 0.0;
    double e2 = 0.0;
    double u_ff = 0.0;
    double u_a = 0.0;
};

// State-feedback + feed-forward position controller (eq. 42) using a filtered
// finite-difference velocity estimate (eq. 46) and output saturation (eq. 44).
// Only the motor angle theta_m is measured.
class Controller {
public:
    Controller(const MotorParams& p, const ControllerGains& g, double T_p)
        : p_(p), g_(g), T_p_(T_p) {}

    ControlOutput update(double theta_m, const RefSample& ref);

private:
    double estimate_velocity(double theta_m);
    double feedforward(const RefSample& ref) const;

    MotorParams p_;
    ControllerGains g_;
    double T_p_;
    double omega_filt_ = 0.0;  // filtered velocity estimate
    double theta_prev_ = 0.0;
    bool init_ = false;
};
