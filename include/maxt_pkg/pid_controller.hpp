/**
 * @file pid_controller.hpp
 * @brief 通用 PID 控制器工具类 (header-only)
 * @date 6-10
 */
#ifndef MAXT_PID_CONTROLLER_HPP
#define MAXT_PID_CONTROLLER_HPP

#include <algorithm>

namespace maxt {

class PIDController {
public:
    PIDController() = default;

    PIDController(double kp, double ki, double kd,
                  double integral_max = 0.0, double output_max = 0.0)
        : kp_(kp), ki_(ki), kd_(kd)
        , integral_max_(integral_max)
        , output_max_(output_max)
    {}

    void setGains(double kp, double ki, double kd) {
        kp_ = kp; ki_ = ki; kd_ = kd;
    }

    void setIntegralClamp(double integral_max) { integral_max_ = integral_max; }
    void setOutputClamp(double output_max)    { output_max_    = output_max; }

    double update(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;

        // P
        p_term_ = kp_ * error;

        // I — 附积分限幅防饱和
        if (ki_ != 0.0 && dt > 0.0) {
            integral_ += error * dt;
            if (integral_max_ > 0.0) {
                integral_ = std::clamp(integral_, -integral_max_, integral_max_);
            }
        }
        i_term_ = ki_ * integral_;

        // D — derivative-on-measurement，避免 setpoint 突变冲击
        if (kd_ != 0.0 && dt > 0.0 && !first_update_) {
            d_term_ = kd_ * (measurement - prev_measurement_) / -dt;
        } else {
            d_term_ = 0.0;
        }

        double output = p_term_ + i_term_ + d_term_;

        if (output_max_ > 0.0) {
            output = std::clamp(output, -output_max_, output_max_);
        }

        prev_error_       = error;
        prev_measurement_ = measurement;
        first_update_     = false;

        return output;
    }

    void reset() {
        prev_error_       = 0.0;
        prev_measurement_ = 0.0;
        integral_         = 0.0;
        first_update_     = true;
        p_term_ = i_term_ = d_term_ = 0.0;
    }

    double pTerm() const { return p_term_; }
    double iTerm() const { return i_term_; }
    double dTerm() const { return d_term_; }

private:
    double kp_{0.0}, ki_{0.0}, kd_{0.0};

    double integral_max_{0.0};
    double output_max_{0.0};

    double prev_error_{0.0};
    double prev_measurement_{0.0};
    double integral_{0.0};
    bool   first_update_{true};

    double p_term_{0.0}, i_term_{0.0}, d_term_{0.0};
};

} // namespace maxt

#endif // MAXT_PID_CONTROLLER_HPP
