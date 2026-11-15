/**
 * @file joy_observation.cpp
 * @brief Построение наблюдаемого снимка джойстика.
 */

#include "control/joy_observation.hpp"

namespace control {

state::JoystickObservation makeObservation(const JoystickSnapshotInput &input,
                                           state::Micros polled_us) noexcept
{
    state::JoystickObservation obs{};
    obs.meta.valid = true;  // снимок получен от работающего сервиса
    obs.meta.measured_us = input.measured_us;
    obs.meta.polled_us = polled_us;
    obs.calibrated = input.calibrated;
    obs.sample_valid = input.sample_valid;
    obs.output_valid = input.output_valid;
    obs.read_fault = input.read_fault;
    obs.neutral = input.neutral;
    obs.stick = input.stick;
    obs.raw = input.raw;
    obs.read_errors = input.read_errors;
    return obs;
}

}  // namespace control
