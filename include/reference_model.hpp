#pragma once

struct ReferenceModelConfig {
    double omega_n = 30.0;  // natural frequency [rad/s]
    double zeta    = 1.0;   // damping ratio
};

class ReferenceModel {
public:
    explicit ReferenceModel(const ReferenceModelConfig& cfg) : cfg_(cfg) {}

    // Advance the model by dt under command r.
    void step(double r, double dt);

    double y() const { return y_; }        // y_m
    double ydot() const { return ydot_; }  // y_m_dot
    void reset(double y0 = 0.0, double ydot0 = 0.0) { y_ = y0; ydot_ = ydot0; }

private:
    ReferenceModelConfig cfg_;
    double y_ = 0.0;
    double ydot_ = 0.0;
};
