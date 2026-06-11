#include "mrac.hpp"
#include <algorithm>

Mrac::Mrac(const MracConfig& cfg)
    : cfg_(cfg),
      model_(cfg.model),
      kx1_(cfg.kx1_0),
      kx2_(cfg.kx2_0),
      kr_(cfg.kr_0) {
    // Solve A_m^T P + P A_m = -Q for the 2x2 reference model
    //   A_m = [[0, 1], [-wn^2, -2 zeta wn]],  Q = diag(q1, q2).
    // Only p12 and p22 enter the adaptive law (e^T P b0 = p12 e1 + p22 e2):
    //   p12 = q1 / (2 wn^2),
    //   p22 = (p12 + q2/2) / (2 zeta wn).
    const double wn = cfg.model.omega_n, z = cfg.model.zeta;
    p12_ = cfg.q1 / (2.0 * wn * wn);
    p22_ = (p12_ + cfg.q2 / 2.0) / (2.0 * z * wn);
}

MracOutput Mrac::update(double theta_m, double omega_m, double r) {
    MracOutput out;
    out.r = r;
    out.omega = omega_m;

    // 1) Reference model: ideal response x_m = [y_m, y_m_dot] to command r.
    model_.step(r, cfg_.T_p);
    out.y_m = model_.y();
    out.ydot_m = model_.ydot();

    // 2) Tracking error e = x - x_m.
    out.e1 = theta_m - out.y_m;
    out.e2 = omega_m - out.ydot_m;

    // 3) sigma = e^T P b0  with  b0 = [0, 1]^T  ->  sigma = p12 e1 + p22 e2.
    const double sigma = p12_ * out.e1 + p22_ * out.e2;

    // 4) Control law u = kx^T x + kr r (current gains), then saturate (eq. 44).
    const double u = kx1_ * theta_m + kx2_ * omega_m + kr_ * r;
    out.u_a = std::clamp(u, -cfg_.u_max, cfg_.u_max);

    // 5) Adaptive laws (Euler-integrated at T_p):
    //    kx_dot = -gx sign(K_p) sigma x / m^2,   kr_dot = -gr sign(K_p) sigma r / m^2.
    // Frozen while the actuator saturates (anti-windup), since the applied
    // voltage then no longer matches the control law the proof assumes.
    const bool saturated = (u != out.u_a);
    if (cfg_.adapt && !saturated) {
        const double m2 = cfg_.normalize
                              ? 1.0 + theta_m * theta_m + omega_m * omega_m + r * r
                              : 1.0;
        const double g = cfg_.Kp_sign * sigma / m2 * cfg_.T_p;
        kx1_ -= cfg_.gamma_x * g * theta_m;
        kx2_ -= cfg_.gamma_x * g * omega_m;
        kr_  -= cfg_.gamma_r * g * r;
    }
    out.kx1 = kx1_;
    out.kx2 = kx2_;
    out.kr = kr_;
    return out;
}
