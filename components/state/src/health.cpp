/**
 * @file health.cpp
 * @brief Возраст, свежесть и здоровье наблюдаемых блоков.
 */

#include "state/health.hpp"

namespace state {

Micros ageUs(Micros now_us, Micros then_us) noexcept
{
    return now_us > then_us ? now_us - then_us : 0U;
}

Freshness freshness(Micros now_us, Micros measured_us, Micros max_age_us) noexcept
{
    if (measured_us == kNever) {
        return Freshness::Never;
    }
    if (now_us < measured_us) {
        return Freshness::Stale;  // сброс счётчика времени или переполнение
    }
    return (now_us - measured_us) <= max_age_us ? Freshness::Fresh : Freshness::Stale;
}

AxisHealth axisHealth(const AxisObservation &obs, Micros now_us,
                      Micros max_age_us) noexcept
{
    if (!obs.meta.valid) {
        return AxisHealth::Unknown;
    }
    if (obs.status.has_fault || obs.status.state == motion::AxisState::Fault) {
        return AxisHealth::Fault;
    }
    if (obs.status.state == motion::AxisState::Disabled) {
        return AxisHealth::Disabled;
    }
    if (obs.meta.polled_us == kNever || now_us < obs.meta.polled_us ||
        ageUs(now_us, obs.meta.polled_us) > max_age_us) {
        return AxisHealth::Stale;
    }
    return AxisHealth::Ok;
}

JoyHealth joyHealth(const JoystickObservation &obs, Micros now_us,
                    Micros max_age_us) noexcept
{
    if (!obs.meta.valid) {
        return JoyHealth::Unknown;
    }
    if (obs.read_fault) {
        return JoyHealth::Fault;
    }
    if (!obs.output_valid) {
        return JoyHealth::Invalid;
    }
    if (freshness(now_us, obs.meta.measured_us, max_age_us) != Freshness::Fresh) {
        return JoyHealth::Stale;
    }
    return JoyHealth::Ok;
}

}  // namespace state
