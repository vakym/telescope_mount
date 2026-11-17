/**
 * @file channels.hpp
 * @brief Общий словарь слоя состояния: источники, фазы, защёлки, причины.
 *
 * Компонент state — чистый C++17 без ESP-IDF. Словарь живёт здесь, чтобы
 * state::ControlObservation не зависел от control, а control подключал те же
 * типы через using.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "motion/types.hpp"

namespace state {

/// Источник намерений управления.
/// Calibration стоит выше Joystick: калибровка опорной точки не должна
/// маскироваться здоровым, но чужим источником.
enum class SourceId : std::uint8_t {
    None = 0,
    Joystick,
    Browser,
    Automation,
    Calibration,
    Count
};

constexpr std::size_t kSourceCount = static_cast<std::size_t>(SourceId::Count);

/// Фаза контура: что делаем с моторами сейчас.
enum class ControlPhase : std::uint8_t {
    Disarmed,           ///< разрешения нет
    WaitingForNeutral,  ///< разрешение есть, ждём свежую подтверждённую нейтраль
    Enabling,           ///< отправлен Enable, ждём подтверждения готовности
    Active,             ///< уставки разрешены; владелец может отсутствовать
    Stopping            ///< активен протокол останова
};

/// Защёлка: почему движение запрещено. Не зависит от фазы.
enum class LatchCode : std::uint8_t {
    None,
    MotorFault,
    DeliveryFailed,
    StopUnconfirmed,
    ConfigInvalid
};

/// Причина, по которой намерение или ввод не приняты.
enum class DenyReason : std::uint8_t {
    None,
    NotCalibrated,
    NoSample,
    InvalidSample,
    ReadFault,
    StaleInput,
    InvalidConfig,
    NotOwner,
    OldSession,
    RepeatedSequence,
    MotorFault,
    DeliveryFailed,
    StopInProgress,
    NotReady,
    Busy,
    /// Требуется подтверждённая нейтраль перед выдачей движения.
    NeutralRequired
};

const char *sourceName(SourceId source) noexcept;
const char *phaseName(ControlPhase phase) noexcept;
const char *latchName(LatchCode latch) noexcept;
const char *denyName(DenyReason reason) noexcept;

/// Имена состояний оси и причин Fault для контракта веб-слоя.
/// Неизвестное значение (испорченная копия, новое значение без обновления
/// словаря) даёт "unknown", а не вымышленное состояние.
const char *axisStateName(motion::AxisState state) noexcept;
const char *faultReasonName(motion::FaultReason reason) noexcept;

}  // namespace state
