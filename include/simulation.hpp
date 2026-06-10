#pragma once
#include "controller.hpp"
#include "motor.hpp"
#include <functional>
#include <string>

struct SimConfig {
    double t_end    = 1.0;     // total simulation time [s]
    double T_p      = 1.0e-3;  // control sample period [s]
    int    substeps = 10;      // integration steps per control period
};

// Sampled-data driver: the controller runs once per period T_p and its output
// voltage is held (ZOH) while the plant is integrated with finer RK4 substeps.
class Simulation {
public:
    // Maps (t, plant state) to the control signals for this step.
    using Control = std::function<ControlOutput(double t, const Motor::State& x)>;

    Simulation(const Motor& motor, const SimConfig& cfg) : motor_(motor), cfg_(cfg) {}

    void run(const Motor::State& x0, const Control& control, const std::string& csv_path);

private:
    Motor motor_;
    SimConfig cfg_;
};
