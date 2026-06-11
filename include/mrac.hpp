#pragma once
#include "reference_model.hpp"

struct MracConfig {
    ReferenceModelConfig model{};   // W_m: omega_n, zeta (desired closed loop)
    double gamma_x = 0.3;           // adaptation rate for the state-feedback gains
    double gamma_r = 0.3;           // adaptation rate for the feed-forward gain
    double q1 = 1.0, q2 = 1.0;      // Lyapunov weight Q = diag(q1, q2)
    double Kp_sign = +1.0;          // sign(K_p) = sign(k_I/(J R)), the only plant info used
    bool   normalize = false;       // divide the update by m^2 = 1 + x^T x + r^2
    double u_max = 12.0;            // control saturation [V] (eq. 44)
    double T_p = 1.0e-3;            // sample period [s]

    double kx1_0 = 0.0, kx2_0 = 0.0, kr_0 = 0.0;  // initial gains
    bool   adapt = true;
};

struct MracOutput {
    double r = 0.0;        // command driving W_m
    double y_m = 0.0;      // reference-model position
    double ydot_m = 0.0;   // reference-model velocity
    double e1 = 0.0;       // theta_m - y_m
    double e2 = 0.0;       // omega_m - y_m_dot
    double kx1 = 0.0, kx2 = 0.0, kr = 0.0;  // adaptive gains
    double omega = 0.0;    // velocity used
    double u_a = 0.0;
};

class Mrac {
public:
    explicit Mrac(const MracConfig& cfg);
    MracOutput update(double theta_m, double omega_m, double r);

private:
    MracConfig cfg_;
    ReferenceModel model_;
    double p12_, p22_;
    double kx1_, kx2_, kr_;
};
