#pragma once
#include <array>

// Physical parameters of the geared PMDC motor (PM1R).
// Defaults are typical values for a motor of this class; refine when real
// numbers are available. Everything is in SI units (rad, rad/s).
struct MotorParams {
    double eta = 1.0 / 51.0;  // gear ratio, link/motor (1:51)
    double R   = 2.0;         // armature resistance [Ohm]
    double k_I = 0.05;        // torque constant [Nm/A]
    double k_E = 0.05;        // back-EMF constant [V s/rad]
    double J_m = 1.0e-5;      // motor inertia [kg m^2]
    double b   = 1.0e-5;      // viscous friction [Nm s/rad]

    // Link (rod), balanced about its rotation axis: its inertia follows from
    // the rod mass and length, J_l = (1/12) m L^2 (uniform rod about centre).
    double rod_mass   = 0.6;  // [kg]
    double rod_length = 1.0;  // [m]

    double J_l() const { return rod_mass * rod_length * rod_length / 12.0; }

    // Inertia reflected to the motor shaft, J = J_m + eta^2 J_l.
    double J() const { return J_m + eta * eta * J_l(); }
};

// Reduced 2nd-order model (armature inductance L = 0, viscous friction),
// eq. (27) of the manual. State is [theta_m, omega_m] on the motor shaft.
class Motor {
public:
    using State = std::array<double, 2>;  // [theta_m, omega_m]

    explicit Motor(const MotorParams& p) : p_(p) {}

    // State derivative for armature voltage u_a and load torque tau_z.
    State derivative(const State& x, double u_a, double tau_z = 0.0) const;

    const MotorParams& params() const { return p_; }
    double link_angle(const State& x) const { return p_.eta * x[0]; }

private:
    MotorParams p_;
};
