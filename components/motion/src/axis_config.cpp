/**
 * @file axis_config.cpp
 * @brief Валидация конфигурации оси.
 */

#include "motion/axis_config.hpp"

#include <cmath>

namespace motion {
namespace {

bool positiveFinite(float value) noexcept
{
    return std::isfinite(value) && value > 0.0F;
}

}  // namespace

bool isValid(const AxisConfig &config) noexcept
{
    if (!positiveFinite(config.start_rate_steps_s)) {
        return false;
    }
    if (!positiveFinite(config.max_rate_steps_s)) {
        return false;
    }
    if (!positiveFinite(config.accel_steps_s2)) {
        return false;
    }
    if (!positiveFinite(config.decel_steps_s2)) {
        return false;
    }
    if (config.min_phase_interval_us == 0 || config.min_alarm_lead_us == 0) {
        return false;
    }
    if (config.max_step_lateness_us == 0) {
        return false;
    }
    if (config.backlash_steps < 0) {
        return false;
    }
    return true;
}

}  // namespace motion
