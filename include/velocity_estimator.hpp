#pragma once

// Drive-speed estimator (manual eq. 46). The rig has no tachometer, only the
// encoder angle theta_m, so omega is reconstructed by a filtered finite
// difference (backward-Euler discretisation of s/(T_f s + 1)):
//
//   omega_f(n) = T_f/(T_f+T_p) * omega_f(n-1) + 1/(T_f+T_p) * (theta(n)-theta(n-1))
class VelocityEstimator {
public:
    VelocityEstimator(double T_f, double T_p) : T_f_(T_f), T_p_(T_p) {}

    double update(double theta) {
        if (!init_) {
            prev_ = theta;
            init_ = true;
        }
        const double dtheta = theta - prev_;
        omega_ = (T_f_ * omega_ + dtheta) / (T_f_ + T_p_);
        prev_ = theta;
        return omega_;
    }

    double value() const { return omega_; }

private:
    double T_f_, T_p_;
    double omega_ = 0.0;
    double prev_ = 0.0;
    bool init_ = false;
};
