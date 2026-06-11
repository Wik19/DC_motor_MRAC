#include "motor.hpp"
#include "mrac.hpp"
#include "simulation.hpp"
#include "trajectory.hpp"
#include <iostream>

// Ideal matching gains for a plant p (manual-derived):
//   kx1* = -wn^2/Kp,  kx2* = (a_p - 2 z wn)/Kp,  kr* = wn^2/Kp.
static void matched_gains(const MotorParams& p, double wn, double z, double& kx1,
                          double& kx2, double& kr) {
    const double JR = p.J() * p.R;
    const double Kp = p.k_I / JR;
    const double a_p = (p.R * p.b + p.k_I * p.k_E) / JR;
    kx1 = -wn * wn / Kp;
    kx2 = (a_p - 2.0 * z * wn) / Kp;
    kr = wn * wn / Kp;
}

int main() {
    Motor motor{MotorParams{}};  // true plant

    SimConfig cfg; // simulation time and step size
    cfg.t_end = 40.0;

    // Multi-sine command: several distinct frequencies make it persistently
    // exciting, so the adaptive gains actually move (a single sine does not).
    TrajectoryConfig tc;
    tc.type = TrajType::MultiSine;
    tc.sine_amps = {6.0, 3.0, 2.0};   // [rad]
    tc.sine_freqs = {1.0, 3.0, 6.0};  // [rad/s], all below wn so W_m passes them
    Trajectory trajectory(tc);

    const double wn = 8.0, z = 1.0;  // reference-model bandwidth

    // The controller is initialised from a NOMINAL design for a lighter 0.15 kg
    // rod; the true rod is ~3x heavier (~1.8x reflected inertia), so the fixed
    // nominal gains mistrack.
    MotorParams nominal;
    nominal.rod_mass = 0.15;
    double kx1_0, kx2_0, kr_0;
    matched_gains(nominal, wn, z, kx1_0, kx2_0, kr_0);

    MracConfig mc;
    mc.model = {wn, z};
    mc.gamma_x = 2;
    mc.gamma_r = 2;
    mc.normalize = false;
    mc.Kp_sign = +1.0;    // the only plant fact used: sign(k_I/(JR)) > 0
    mc.T_p = cfg.T_p;
    mc.kx1_0 = kx1_0;
    mc.kx2_0 = kx2_0;
    mc.kr_0 = kr_0;

    const char* header = "r,y_m,ydot_m,e1,e2,kx1,kx2,kr,omega";

    // omega_m is taken from the true plant state x[1] (full-state MRAC).
    auto wrap = [&](Mrac& c) {
        return [&c, &trajectory](double t, const Motor::State& x) -> StepResult {
            const MracOutput o = c.update(x[0], x[1], trajectory.eval(t).pos);
            return {o.u_a,
                    {o.r, o.y_m, o.ydot_m, o.e1, o.e2, o.kx1, o.kx2, o.kr, o.omega}};
        };
    };

    // Fixed nominal controller (adaptation off) for comparison.
    MracConfig fc = mc;
    fc.adapt = false;
    Mrac fixed_ctrl(fc);
    Simulation(motor, cfg).run({0.0, 0.0}, wrap(fixed_ctrl), "mrac_fixed.csv", header);

    // Adaptive MRAC.
    Mrac mrac(mc);
    Simulation(motor, cfg).run({0.0, 0.0}, wrap(mrac), "mrac.csv", header);

    double k1, k2, kr;
    matched_gains(motor.params(), wn, z, k1, k2, kr);
    std::cout << "MRAC demo: nominal (" << nominal.rod_mass << " kg rod) controller on true ("
              << motor.params().rod_mass << " kg) plant\n"
              << "  start gains : kx1=" << kx1_0 << " kx2=" << kx2_0 << " kr=" << kr_0
              << "\n"
              << "  ideal(true) : kx1=" << k1 << " kx2=" << k2 << " kr=" << kr << "\n"
              << "  wrote mrac_fixed.csv (adaptation off) and mrac.csv (adaptation on)\n";
    return 0;
}
