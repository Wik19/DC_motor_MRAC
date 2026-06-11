#pragma once
#include "motor.hpp"
#include <functional>
#include <string>
#include <vector>

struct SimConfig {
    double t_end    = 5.0;     // total simulation time [s]
    double T_p      = 1.0e-3;  // control sample period [s]
    int    substeps = 10;      // integration steps per control period
};

struct StepResult {
    double u_a = 0.0;
    std::vector<double> log;
};

// Sampled-data driver: the controller runs once per period T_p and its output
// voltage is held (ZOH) while the plant is integrated with finer RK4 substeps.
class Simulation {
public:
    using Control = std::function<StepResult(double t, const Motor::State& x)>;

    Simulation(const Motor& motor, const SimConfig& cfg) : motor_(motor), cfg_(cfg) {}

    void run(const Motor::State& x0, const Control& control,
             const std::string& csv_path, const std::string& extra_header);

private:
    Motor motor_;
    SimConfig cfg_;
};
