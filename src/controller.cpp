#include "controller.hpp"
#include <algorithm>

ControllerGains place_gains(const MotorParams& p, double omega_n, double zeta) {
    ControllerGains g;
    const double JR = p.J() * p.R;
    // From eq. (41): k_I k1/(JR) = wn^2  and  (R b + k_I k_E + k_I k2)/(JR) = 2 zeta wn.
    g.k1 = omega_n * omega_n * JR / p.k_I;
    g.k2 = (2.0 * zeta * omega_n * JR - p.R * p.b - p.k_I * p.k_E) / p.k_I;
    return g;
}

double Controller::feedforward(const TrajPoint& ref) const {
    // Feed-forward voltage, eq. (39) with viscous friction (f_C = 0):
    //   u_FF = (R/k_I) [ J theta_md_ddot + (R b + k_I k_E)/R * theta_md_dot ]
    const double J = p_.J();
    return (p_.R / p_.k_I) *
           (J * ref.acc + (p_.R * p_.b + p_.k_I * p_.k_E) / p_.R * ref.vel);
}

ControlOutput Controller::update(double theta_m, const TrajPoint& ref) {
    ControlOutput out;
    out.ref_pos = ref.pos;
    out.ref_vel = ref.vel;
    out.omega_est = vel_.update(theta_m);

    // Tracking error, eq. (43): e1 = theta_md - theta_m, e2 = theta_md_dot - omega.
    out.e1 = ref.pos - theta_m;
    out.e2 = ref.vel - out.omega_est;
    out.u_ff = g_.feedforward ? feedforward(ref) : 0.0;

    // Control law, eq. (42): u = k1 e1 + k2 e2 + u_FF, then saturate (eq. 44).
    const double u = g_.k1 * out.e1 + g_.k2 * out.e2 + out.u_ff;
    out.u_a = std::clamp(u, -g_.u_max, g_.u_max);
    return out;
}
