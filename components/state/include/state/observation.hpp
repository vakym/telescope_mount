/**
 * @file observation.hpp
 * @brief Наблюдаемые снимки системы: оси, джойстик, контур управления.
 *
 * Блоки — POD-копии. Хаб публикует их под одним мьютексом; свежесть
 * измерения считается по времени источника, а не по времени публикации.
 */

#pragma once

#include <cstdint>

#include "input/joystick_config.hpp"
#include "motion/axis_planner.hpp"
#include "state/channels.hpp"

namespace state {

using Micros = std::uint64_t;

/// Отсутствие отметки времени: отличается от «нулевого возраста».
constexpr Micros kNever = 0;

/// Метаданные любого наблюдаемого блока. Три времени раздельны.
struct ObservationMeta {
    Micros measured_us = kNever;   ///< время исходного измерения, если известно
    Micros polled_us = kNever;     ///< время опроса источника агрегатором
    Micros published_us = kNever;  ///< время записи блока в хаб
    std::uint32_t sequence = 0;    ///< номер публикации, назначает хаб
    bool valid = false;            ///< была хотя бы одна публикация
};

/// Снимок оси: подтверждённое исполнителем состояние плюс метаданные опроса.
struct AxisObservation {
    ObservationMeta meta{};
    motion::AxisStatus status{};
    std::uint32_t read_failures = 0;  ///< неудачных опросов подряд
};

/// Снимок джойстика. measured_us — время последнего валидного отсчёта.
struct JoystickObservation {
    ObservationMeta meta{};
    bool calibrated = false;
    bool sample_valid = false;
    bool output_valid = false;
    bool read_fault = false;
    bool neutral = true;
    input::StickState stick{};
    input::RawSample raw{};
    std::uint32_t read_errors = 0;
};

/// Состояние контура управления. Пишет только управляющая задача.
struct ControlObservation {
    ObservationMeta meta{};  ///< measured_us = kNever: это не измерение
    ControlPhase phase = ControlPhase::Disarmed;
    LatchCode latch = LatchCode::None;
    SourceId owner = SourceId::None;
    std::uint32_t owner_generation = 0;
    bool permit = false;
    bool recovery_active = false;
    DenyReason deny = DenyReason::None;

    float desired_rate[2] = {0.0F, 0.0F};    ///< индекс = motion::AxisId
    float enqueued_rate[2] = {0.0F, 0.0F};   ///< последняя принятая в очередь
    bool enqueued_known[2] = {false, false};
    float confirmed_rate[2] = {0.0F, 0.0F};
    std::uint64_t last_ticket_id[2] = {0, 0};
    std::uint64_t last_applied_id[2] = {0, 0};
    std::uint32_t command_epoch[2] = {0, 0};

    Micros stop_requested_us = kNever;
    bool stop_confirmed = false;
    std::uint8_t stop_reason = 0;
    std::uint32_t delivery_failures[2] = {0, 0};
    std::uint32_t stop_timeouts = 0;
    std::uint32_t escalations = 0;
    std::uint32_t ack_mismatches = 0;
    std::uint32_t emergency_slot_busy = 0;
    std::uint32_t unmatched_emergency_notifications = 0;
    std::uint32_t notify_failures = 0;
    std::uint32_t rejected_intents = 0;
    std::uint64_t cycle = 0;
};

/// Типизированное расширение датчиками: новое значение — новый тип,
/// а не словарь и не динамический реестр.
template <typename T>
struct SensorObservation {
    ObservationMeta meta{};
    T value{};
};

/// Согласованный снимок для диагностики и будущего UI.
struct SystemSnapshot {
    Micros generated_us = kNever;
    AxisObservation axis[2]{};  ///< индекс = motion::AxisId
    JoystickObservation joystick{};
    ControlObservation control{};
};

}  // namespace state
