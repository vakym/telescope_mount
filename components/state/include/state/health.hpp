/**
 * @file health.hpp
 * @brief Правила возраста, свежести и здоровья наблюдаемых блоков.
 *
 * Копирование блока в хаб не делает данные свежими: свежесть считается по
 * времени измерения, а для осей — по времени опроса.
 */

#pragma once

#include <cstdint>

#include "state/observation.hpp"

namespace state {

/// Насыщающая разность now - then; при now < then возвращает 0 и вызывающий
/// считает данные устаревшими.
Micros ageUs(Micros now_us, Micros then_us) noexcept;

enum class Freshness : std::uint8_t { Never, Fresh, Stale };

/// Никогда не возвращает Fresh при measured_us == kNever.
Freshness freshness(Micros now_us, Micros measured_us, Micros max_age_us) noexcept;

enum class AxisHealth : std::uint8_t { Unknown, Ok, Stale, Fault, Disabled };

AxisHealth axisHealth(const AxisObservation &obs, Micros now_us,
                      Micros max_age_us) noexcept;

enum class JoyHealth : std::uint8_t { Unknown, Ok, Invalid, Fault, Stale };

/// Порядок: read_fault -> Invalid (output_valid == false) -> Stale -> Ok.
JoyHealth joyHealth(const JoystickObservation &obs, Micros now_us,
                    Micros max_age_us) noexcept;

/// Текстовые имена для контракта веб-слоя и диагностики.
const char *freshnessName(Freshness value) noexcept;
const char *axisHealthName(AxisHealth value) noexcept;
const char *joyHealthName(JoyHealth value) noexcept;

}  // namespace state
