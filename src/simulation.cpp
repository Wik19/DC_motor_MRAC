#include "simulation.hpp"
#include "integrator.hpp"
#include <fstream>

void Simulation::run(const Motor::State& x0, const Control& control,
                     const std::string& csv_path) {
    std::ofstream out(csv_path);
    out << "t,theta_md,theta_m,theta_l,omega_md,omega_m,omega_est,e1,e2,u_ff,u_a\n";

    Motor::State x = x0;
    const double dt = cfg_.T_p / cfg_.substeps;
    const long ticks = static_cast<long>(cfg_.t_end / cfg_.T_p);

    for (long n = 0; n <= ticks; ++n) {
        const double t = n * cfg_.T_p;
        const ControlOutput c = control(t, x);
        out << t << ',' << c.ref_pos << ',' << x[0] << ',' << motor_.link_angle(x) << ','
            << c.ref_vel << ',' << x[1] << ',' << c.omega_est << ',' << c.e1 << ','
            << c.e2 << ',' << c.u_ff << ',' << c.u_a << '\n';

        if (n == ticks) break;
        for (int s = 0; s < cfg_.substeps; ++s) {
            auto f = [&](const Motor::State& xs) { return motor_.derivative(xs, c.u_a); };
            x = rk4_step(x, dt, f);
        }
    }
}
