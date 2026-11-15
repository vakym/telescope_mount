/**
 * @file joystick_processor.cpp
 * @brief Калибровка центра, EMA, нормализация и радиальная мёртвая зона.
 */

#include "input/joystick_processor.hpp"

#include <algorithm>
#include <cmath>

namespace input {
namespace {

bool finiteAxis(const AxisCalibration &axis) noexcept
{
    return std::isfinite(axis.center) && std::isfinite(axis.min) &&
           std::isfinite(axis.max) && axis.min < axis.max &&
           axis.center > axis.min && axis.center < axis.max;
}

std::size_t indexOf(StickAxis axis) noexcept
{
    return static_cast<std::size_t>(axis);
}

}  // namespace

bool isValid(const JoystickConfig &config) noexcept
{
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        if (!finiteAxis(config.axis[i])) {
            return false;
        }
    }
    if (!std::isfinite(config.dead_zone) || config.dead_zone <= 0.0F ||
        config.dead_zone >= 1.0F) {
        return false;
    }
    if (!std::isfinite(config.dead_zone_exit) || config.dead_zone_exit < 0.0F ||
        config.dead_zone_exit >= config.dead_zone) {
        return false;
    }
    if (!std::isfinite(config.filter_alpha) || config.filter_alpha <= 0.0F ||
        config.filter_alpha > 1.0F) {
        return false;
    }
    return true;
}

bool CenterCalibrator::configure(std::size_t samples, float noise_limit,
                                 float center_min, float center_max) noexcept
{
    if (samples == 0 || !std::isfinite(noise_limit) || noise_limit < 0.0F ||
        !std::isfinite(center_min) || !std::isfinite(center_max) ||
        center_min >= center_max) {
        configured_ = false;
        return false;
    }

    samples_ = samples;
    noise_limit_ = noise_limit;
    center_min_ = center_min;
    center_max_ = center_max;
    configured_ = true;
    reset();
    return true;
}

void CenterCalibrator::reset() noexcept
{
    count_ = 0;
    sum_[0] = 0.0;
    sum_[1] = 0.0;
    min_[0] = 0;
    min_[1] = 0;
    max_[0] = 0;
    max_[1] = 0;
    have_sample_ = false;
    complete_ = false;
    stable_ = false;
}

bool CenterCalibrator::add(const RawSample &sample) noexcept
{
    if (!configured_ || !sample.valid || complete_) {
        return false;
    }

    for (std::size_t i = 0; i < kAxisCount; ++i) {
        const std::uint16_t value = sample.counts[i];
        if (!have_sample_) {
            min_[i] = value;
            max_[i] = value;
        } else {
            min_[i] = std::min(min_[i], value);
            max_[i] = std::max(max_[i], value);
        }
        sum_[i] += static_cast<double>(value);
    }
    have_sample_ = true;
    ++count_;

    if (count_ >= samples_) {
        complete_ = true;
        stable_ = withinBounds() && quietEnough();
    }
    return true;
}

bool CenterCalibrator::ready() const noexcept
{
    return complete_;
}

bool CenterCalibrator::stable() const noexcept
{
    return stable_;
}

float CenterCalibrator::center(StickAxis axis) const noexcept
{
    if (count_ == 0) {
        return 0.0F;
    }
    const std::size_t idx = indexOf(axis);
    return static_cast<float>(sum_[idx] / static_cast<double>(count_));
}

bool CenterCalibrator::withinBounds() const noexcept
{
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        const float mean = static_cast<float>(sum_[i] / static_cast<double>(count_));
        if (mean < center_min_ || mean > center_max_) {
            return false;
        }
    }
    return true;
}

bool CenterCalibrator::quietEnough() const noexcept
{
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        const float peak_to_peak =
            static_cast<float>(max_[i]) - static_cast<float>(min_[i]);
        if (peak_to_peak > noise_limit_) {
            return false;
        }
    }
    return true;
}

bool JoystickProcessor::configure(const JoystickConfig &config) noexcept
{
    if (!isValid(config)) {
        configured_ = false;
        return false;
    }
    config_ = config;
    configured_ = true;
    reset();
    return true;
}

void JoystickProcessor::reset() noexcept
{
    if (!configured_) {
        current_ = StickState{};
        neutral_ = true;
        return;
    }
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        filtered_[i] = config_.axis[i].center;
    }
    neutral_ = true;
    current_ = makeState(false);
}

StickState JoystickProcessor::resetWithSample(const RawSample &sample) noexcept
{
    if (!configured_ || !sample.valid) {
        return current_;
    }
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        filtered_[i] = static_cast<float>(sample.counts[i]);
    }
    // История гистерезиса сбрасывается в нейтраль, затем применяется
    // обычный порог входа. Результат не зависит от состояния до сбоя.
    neutral_ = true;
    current_ = makeState(false);
    return current_;
}

StickState JoystickProcessor::update(const RawSample &sample) noexcept
{
    if (!configured_ || !sample.valid) {
        return current_;
    }

    const float alpha = config_.filter_alpha;
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        const float value = static_cast<float>(sample.counts[i]);
        filtered_[i] = alpha * value + (1.0F - alpha) * filtered_[i];
    }
    current_ = makeState(true);
    return current_;
}

float JoystickProcessor::normalize(StickAxis axis) const noexcept
{
    const std::size_t idx = indexOf(axis);
    const AxisCalibration &cal = config_.axis[idx];
    const float value = filtered_[idx];

    float normalized = 0.0F;
    if (value >= cal.center) {
        normalized = (value - cal.center) / (cal.max - cal.center);
    } else {
        normalized = (value - cal.center) / (cal.center - cal.min);
    }
    normalized = std::clamp(normalized, -1.0F, 1.0F);
    return cal.invert ? -normalized : normalized;
}

StickState JoystickProcessor::makeState(bool apply_hysteresis) noexcept
{
    StickState state;
    for (std::size_t i = 0; i < kAxisCount; ++i) {
        state.filtered[i] = filtered_[i];
        state.normalized[i] = normalize(static_cast<StickAxis>(i));
    }

    const float x = state.normalized[indexOf(StickAxis::X)];
    const float y = state.normalized[indexOf(StickAxis::Y)];
    const float length = std::sqrt(x * x + y * y);
    state.magnitude = std::min(1.0F, length);

    if (apply_hysteresis) {
        if (neutral_ && state.magnitude >= config_.dead_zone) {
            neutral_ = false;
        } else if (!neutral_ && state.magnitude <= config_.dead_zone_exit) {
            neutral_ = true;
        }
    } else {
        neutral_ = state.magnitude < config_.dead_zone;
    }

    state.neutral = neutral_;
    current_ = state;
    return state;
}

}  // namespace input
