/**
 * @file joy_mapping.hpp
 * @brief Детерминированное преобразование снимка джойстика в уставки осей.
 *
 * Чистая функция: снимок + конфигурация + время -> две уставки или причина
 * отказа. Никаких обращений к сервисам, никакой второй EMA и никакого
 * второго профиля ускорения.
 */

#pragma once

#include <cmath>

#include "motion/types.hpp"
#include "state/channels.hpp"
#include "state/health.hpp"
#include "state/observation.hpp"

namespace control {

using state::DenyReason;
using state::Micros;

struct JoyMappingConfig {
    motion::AxisId x_axis = motion::AxisId::Azimuth;
    motion::AxisId y_axis = motion::AxisId::Altitude;
    /// Резерв для стендовой ориентации. По умолчанию false: физическая
    /// инверсия выполняется в input через CONFIG_JOY_INVERT_X/Y.
    bool invert_x = false;
    bool invert_y = false;
    /// Индекс = motion::AxisId, полушагов/с.
    float max_rate[2] = {800.0F, 800.0F};
    /// Пол ненулевой уставки; должен быть меньше max_rate.
    float min_run_rate[2] = {40.0F, 40.0F};
    /// Компонентный гейт, совпадает с CONFIG_JOY_DEAD_ZONE_PERMILLE.
    float dead_zone = 0.12F;
    /// Возраст измерения, после которого ввод считается устаревшим.
    Micros stale_after_us = 200'000;
};

/// Проверяет конфигурацию; невалидная — ошибка запуска, а не подстановка.
bool isValid(const JoyMappingConfig &config) noexcept;

struct JoyConversion {
    bool ok = false;
    DenyReason deny = DenyReason::None;
    float rate[2] = {0.0F, 0.0F};
    bool neutral = false;
};

JoyConversion convert(const state::JoystickObservation &snapshot,
                      const JoyMappingConfig &config, Micros now_us) noexcept;

}  // namespace control
