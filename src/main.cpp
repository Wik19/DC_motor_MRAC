#include "controller.hpp"
#include "motor.hpp"
#include "reference_model.hpp"
#include "simulation.hpp"
#include <iostream>

int main() {
    Motor motor{MotorParams{}};
    const MotorParams& p = motor.params();

    SimConfig cfg;
    cfg.t_end = 5.0;

    // Closed-loop tracking-error poles (natural frequency / damping).
    const double omega_n = 30.0, zeta = 1.0;
    ControllerGains gains = place_gains(p, omega_n, zeta);
    Controller controller(p, gains, cfg.T_p);

    // Sinusoidal reference on the motor shaft.
    ReferenceConfig rc;
    rc.type = RefType::Sine;
    rc.amplitude = 50.0;  // [rad]
    rc.omega = 3.0;       // [rad/s]
    ReferenceModel reference(rc);

    auto control = [&](double t, const Motor::State& x) {
        return controller.update(x[0], reference.eval(t));
    };

    Simulation sim(motor, cfg);
    sim.run({0.0, 0.0}, control, "closedloop.csv");

    std::cout << "Closed-loop tracking (sine reference)\n"
              << "  poles: omega_n=" << omega_n << " rad/s, zeta=" << zeta << '\n'
              << "  gains: k1=" << gains.k1 << ", k2=" << gains.k2 << '\n'
              << "  Wrote closedloop.csv\n";
    return 0;
}
