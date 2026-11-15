/**
 * @file joy_mapping.cpp
 * @brief Преобразование положения стика в уставки осей.
 *
 * Формула: rate = sign(n) * (r_min + (r_max - r_min) * scaled), где
 * scaled = clamp((|n| - dz) / (1 - dz), 0, 1). При |n| <= dz компонент равен
 * нулю; при |n| чуть больше dz уставка скачком достигает r_min.
 */

#include "control/joy_mapping.hpp"

#include <algorithm>
#include <cstddef>

namespace control {
namespace {

std::size_t indexOf(motion::AxisId id) noexcept
{
    return static_cast<std::size_t>(id);
}

bool axisIdValid(motion::AxisId id) noexcept
{
    return id == motion::AxisId::Azimuth || id == motion::AxisId::Altitude;
}

float componentRate(float normalized, float max_rate, float min_run_rate,
                    float dead_zone) noexcept
{
    const float magnitude = std::min(std::fabs(normalized), 1.0F);
    if (magnitude <= dead_zone) {
        return 0.0F;
    }
    const float scaled =
        std::clamp((magnitude - dead_zone) / (1.0F - dead_zone), 0.0F, 1.0F);
    const float rate = min_run_rate + (max_rate - min_run_rate) * scaled;
    return std::copysign(rate, normalized);
}

}  // namespace

bool isValid(const JoyMappingConfig &config) noexcept
{
    if (!axisIdValid(config.x_axis) || !axisIdValid(config.y_axis) ||
        config.x_axis == config.y_axis) {
        return false;
    }
    if (!std::isfinite(config.dead_zone) || config.dead_zone <= 0.0F ||
        config.dead_zone >= 1.0F) {
        return false;
    }
    if (config.stale_after_us == 0U) {
        return false;
    }
    for (std::size_t i = 0; i < 2; ++i) {
        if (!std::isfinite(config.max_rate[i]) || !std::isfinite(config.min_run_rate[i])) {
            return false;
        }
        if (config.max_rate[i] <= 0.0F || config.min_run_rate[i] < 0.0F ||
            config.min_run_rate[i] >= config.max_rate[i]) {
            return false;
        }
    }
    return true;
}

JoyConversion convert(const state::JoystickObservation &snapshot,
                      const JoyMappingConfig &config, Micros now_us) noexcept
{
    JoyConversion out{};
    if (!isValid(config)) {
        out.deny = DenyReason::InvalidConfig;
        return out;
    }
    if (!snapshot.meta.valid) {
        out.deny = DenyReason::NoSample;
        return out;
    }
    if (snapshot.read_fault) {
        out.deny = DenyReason::ReadFault;
        return out;
    }
    if (!snapshot.output_valid) {
        out.deny = DenyReason::InvalidSample;
        return out;
    }
    if (!snapshot.calibrated) {
        out.deny = DenyReason::NotCalibrated;
        return out;
    }
    if (state::freshness(now_us, snapshot.meta.measured_us, config.stale_after_us) !=
        state::Freshness::Fresh) {
        out.deny = DenyReason::StaleInput;
        return out;
    }

    const float nx = snapshot.stick.normalized[static_cast<std::size_t>(input::StickAxis::X)];
    const float ny = snapshot.stick.normalized[static_cast<std::size_t>(input::StickAxis::Y)];
    if (!std::isfinite(nx) || !std::isfinite(ny)) {
        out.deny = DenyReason::InvalidSample;
        return out;
    }

    if (snapshot.neutral) {
        out.ok = true;
        out.neutral = true;
        return out;
    }

    const float x = config.invert_x ? -nx : nx;
    const float y = config.invert_y ? -ny : ny;
    const std::size_t x_index = indexOf(config.x_axis);
    const std::size_t y_index = indexOf(config.y_axis);
    out.rate[x_index] =
        componentRate(x, config.max_rate[x_index], config.min_run_rate[x_index],
                      config.dead_zone);
    out.rate[y_index] =
        componentRate(y, config.max_rate[y_index], config.min_run_rate[y_index],
                      config.dead_zone);
    out.ok = true;
    out.neutral = false;
    return out;
}

}  // namespace control
