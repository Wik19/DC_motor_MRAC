#include "simulation.hpp"
#include "integrator.hpp"
#include <fstream>

void Simulation::run(const Motor::State& x0, const Control& control,
                     const std::string& csv_path, const std::string& extra_header) {
    std::ofstream out(csv_path);
    out << "t,theta_m,omega_m,theta_l,u_a," << extra_header << "\n";

    Motor::State x = x0;
    const double dt = cfg_.T_p / cfg_.substeps;
    const long ticks = static_cast<long>(cfg_.t_end / cfg_.T_p);

    for (long n = 0; n <= ticks; ++n) {
        const double t = n * cfg_.T_p;
        const StepResult c = control(t, x);
        out << t << ',' << x[0] << ',' << x[1] << ',' << motor_.link_angle(x) << ','
            << c.u_a;
        for (double v : c.log) out << ',' << v;
        out << '\n';

        if (n == ticks) break;
        for (int s = 0; s < cfg_.substeps; ++s) {
            auto f = [&](const Motor::State& xs) { return motor_.derivative(xs, c.u_a); };
            x = rk4_step(x, dt, f);
        }
    }
}
