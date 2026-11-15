/**
 * @file joy_observation.hpp
 * @brief Чистое построение наблюдаемого снимка джойстика.
 *
 * Валидность наблюдения (meta.valid) означает «снимок получен от работающего
 * сервиса» и НЕ равна валидности отсчёта: sample_valid/output_valid/read_fault
 * переносятся как есть. Без явной установки valid снимок навсегда остаётся
 * невалидным для convert(), хотя хаб показывает его как опубликованный.
 */

#pragma once

#include "input/joystick_config.hpp"
#include "state/observation.hpp"

namespace control {

/// Поля, которые берутся из input_hal::JoystickStatus без изменений.
struct JoystickSnapshotInput {
    bool calibrated = false;
    bool sample_valid = false;
    bool output_valid = false;
    bool read_fault = false;
    bool neutral = true;
    input::StickState stick{};
    input::RawSample raw{};
    state::Micros measured_us = state::kNever;
    std::uint32_t read_errors = 0;
};

/// Строит наблюдение с meta.valid = true и заданным временем опроса.
state::JoystickObservation makeObservation(const JoystickSnapshotInput &input,
                                           state::Micros polled_us) noexcept;

}  // namespace control
