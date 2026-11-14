/**
 * @file deadline_policy.hpp
 * @brief Чистые функции расписания: проверка пробуждения и планирование alarm.
 *
 * Две независимые проверки:
 *  - classifyWakeup() решает, исполнять ли уже наступившее событие таймера;
 *  - planNextStep() решает, можно ли назначить следующий alarm и с каким
 *    дедлайном.
 *
 * Время передаётся в единицах StepClock (при разрешении 1 МГц они совпадают
 * с микросекундами).
 */

#pragma once

#include <cstdint>

#include "motion/types.hpp"

namespace motion {

/// Решение по уже наступившему событию таймера.
enum class StepDue : std::uint8_t {
    ExecuteNow,      ///< ожидающее событие исполняется
    IgnoreSpurious,  ///< событие раньше дедлайна: шаг не исполняется
    IgnoreStale,     ///< ожидающего события нет: уведомление устарело
    MissedTooFar,    ///< опоздание больше max_lateness, действует политика
};

/**
 * @brief Классифицирует пробуждение по ожидающему событию.
 *
 * Шаг никогда не исполняется раньше дедлайна: допуск опоздания здесь не
 * участвует, иначе устаревшее уведомление могло бы исполниться на новом
 * движении.
 */
StepDue classifyWakeup(std::uint64_t now_us,
                       std::uint64_t pending_deadline_us,
                       bool has_pending,
                       std::uint64_t max_lateness_us) noexcept;

/// Решение по следующему alarm.
enum class DeadlineAction : std::uint8_t { ArmAtDeadline, ReplanFromNow, Fault };

struct DeadlineDecision {
    DeadlineAction action = DeadlineAction::ArmAtDeadline;
    std::uint64_t deadline_us = 0;  ///< используется вызывающим всегда
    bool count_correction = false;
    bool count_replan = false;
};

/**
 * @brief Вычисляет дедлайн следующего события.
 *
 * @param now_us                текущее время
 * @param next_deadline_us      дедлайн из профиля (не меньше последнего)
 * @param last_switch_us        время последнего физического переключения
 * @param interval_us           интервал до следующего события по профилю
 * @param min_phase_interval_us минимальный интервал между переключениями
 * @param min_alarm_lead_us     минимальный запас до будущего alarm
 * @param late_tolerance_us     допуск малой коррекции графика
 * @param policy                политика при большой просрочке
 *
 * При коррекции возвращается сдвинутый дедлайн, при полноценном Replan —
 * не раньше now + interval. Ветка Fault не предназначена для вызова armAt().
 */
DeadlineDecision planNextStep(std::uint64_t now_us,
                              std::uint64_t next_deadline_us,
                              std::uint64_t last_switch_us,
                              std::uint32_t interval_us,
                              std::uint32_t min_phase_interval_us,
                              std::uint32_t min_alarm_lead_us,
                              std::uint32_t late_tolerance_us,
                              DeadlineMissPolicy policy) noexcept;

}  // namespace motion
