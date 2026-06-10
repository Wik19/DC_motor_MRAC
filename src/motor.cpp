#include "motor.hpp"

Motor::State Motor::derivative(const State& x, double u_a, double tau_z) const {
    const double omega = x[1];
    const double J = p_.J();

    // omega_dot = -a*omega + K*u_a - (eta/J)*tau_z
    const double a = (p_.R * p_.b + p_.k_I * p_.k_E) / (J * p_.R);
    const double K = p_.k_I / (J * p_.R);

    const double omega_dot = -a * omega + K * u_a - (p_.eta / J) * tau_z;
    return {omega, omega_dot};
}
