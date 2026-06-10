#include "controller.hpp"
#include <algorithm>

ControllerGains place_gains(const MotorParams& p, double omega_n, double zeta) {
    ControllerGains g;
    const double JR = p.J() * p.R;
    g.k1 = omega_n * omega_n * JR / p.k_I;
    g.k2 = (2.0 * zeta * omega_n * JR - p.R * p.b - p.k_I * p.k_E) / p.k_I;
    return g;
}

double Controller::estimate_velocity(double theta_m) {
    if (!init_) {
        theta_prev_ = theta_m;
        init_ = true;
    }
    const double dtheta = theta_m - theta_prev_;
    omega_filt_ = (g_.T_f * omega_filt_ + dtheta) / (g_.T_f + T_p_);
    theta_prev_ = theta_m;
    return omega_filt_;
}

double Controller::feedforward(const RefSample& ref) const {
    const double J = p_.J();
    return (p_.R / p_.k_I) *
           (J * ref.acc + (p_.R * p_.b + p_.k_I * p_.k_E) / p_.R * ref.vel);
}

ControlOutput Controller::update(double theta_m, const RefSample& ref) {
    ControlOutput out;
    out.ref_pos = ref.pos;
    out.ref_vel = ref.vel;
    out.omega_est = estimate_velocity(theta_m);
    out.e1 = ref.pos - theta_m;
    out.e2 = ref.vel - out.omega_est;
    out.u_ff = g_.feedforward ? feedforward(ref) : 0.0;

    const double u = g_.k1 * out.e1 + g_.k2 * out.e2 + out.u_ff;
    out.u_a = std::clamp(u, -g_.u_max, g_.u_max);
    return out;
}
