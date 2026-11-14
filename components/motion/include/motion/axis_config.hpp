/**
 * @file axis_config.hpp
 * @brief Конфигурация одной оси и её валидация.
 */

#pragma once

#include <cstdint>

#include "motion/types.hpp"

namespace motion {

/// Политика удержания после достижения нулевой скорости.
struct HoldPolicy {
    bool hold_enabled = false;  ///< false = обесточивать сразу после останова
    /// Действует только при hold_enabled = true.
    /// 0 = удерживать бессрочно до Release, Disable или новой уставки.
    std::uint32_t hold_timeout_ms = 0;
};

/// Параметры профиля и расписания одной оси.
struct AxisConfig {
    AxisId id = AxisId::Azimuth;
    StepPattern pattern = StepPattern::Half8;

    // Скорости в полушагах/с на границе шага.
    float start_rate_steps_s = 100.0F;  ///< скорость первого шага, не нижний предел
    float max_rate_steps_s = 800.0F;
    float accel_steps_s2 = 800.0F;
    float decel_steps_s2 = 800.0F;

    std::uint32_t reversal_pause_us = 0;        ///< пауза между торможением и реверсом
    std::uint32_t min_phase_interval_us = 600;  ///< между физическими переключениями
    std::uint32_t min_alarm_lead_us = 50;       ///< запас до будущего alarm
    std::uint32_t late_tolerance_us = 200;      ///< допуск коррекции графика
    std::uint32_t max_step_lateness_us = 2000;  ///< порог действия политики

    bool invert_direction = false;
    HoldPolicy hold{};
    DeadlineMissPolicy deadline_miss = DeadlineMissPolicy::Replan;
    std::int32_t backlash_steps = 0;  ///< резерв, компенсация люфта, пока 0
};

/**
 * @brief Проверяет конфигурацию перед запуском.
 *
 * Невалидная конфигурация — ошибка инициализации, а не повод молча
 * скорректировать значения.
 */
bool isValid(const AxisConfig &config) noexcept;

}  // namespace motion
