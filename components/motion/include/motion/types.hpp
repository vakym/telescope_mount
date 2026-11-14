/**
 * @file types.hpp
 * @brief Базовые типы слоя движения.
 *
 * Компонент motion — чистая логика C++17 без зависимости от ESP-IDF.
 * Единица счёта во всех внутренних структурах — полушаг (half-step):
 * 4096 полушагов на оборот выходного вала 28BYJ-48 (номинал).
 */

#pragma once

#include <cstdint>

namespace motion {

/// Оси монтировки. Альт-азимутальная схема по docs/hardware/overview.md.
enum class AxisId : std::uint8_t { Azimuth = 0, Altitude = 1, Count = 2 };

constexpr std::uint8_t axisCount() noexcept
{
    return static_cast<std::uint8_t>(AxisId::Count);
}

/// Направление вращения по возрастанию или убыванию логической координаты.
enum class Direction : std::int8_t { Backward = -1, Forward = +1 };

/// Режимы обмоток, реализуемые на ULN2003 (микрошаг недоступен).
enum class StepPattern : std::uint8_t {
    Half8,        ///< 8 состояний, 1 полушаг на переключение
    Full4Double,  ///< 2 фазы включены, 2 полушага на переключение
    Full4Wave,    ///< 1 фаза включена, 2 полушага на переключение
};

/// Сколько полушагов даёт одно физическое переключение фаз.
constexpr std::int32_t deltaHalfSteps(StepPattern pattern) noexcept
{
    return pattern == StepPattern::Half8 ? 1 : 2;
}

/// Состояние оси.
enum class AxisState : std::uint8_t {
    Disabled,      ///< ось выключена, команды движения игнорируются
    Idle,          ///< стоит, обмотки обесточены
    Accelerating,
    Cruising,
    Decelerating,
    Holding,       ///< стоит, обмотки под током
    Released,      ///< стоит, обмотки обесточены после удержания
    Fault,         ///< ошибка расписания или исполнения
};

/// Как останавливать движение.
enum class StopMode : std::uint8_t {
    Smooth,     ///< торможение с заданным decel, затем политика удержания
    Immediate,  ///< ожидающий шаг отменяется, обмотки выключаются
};

/// Что делать, когда просрочка уже не позволяет выдать шаг по расписанию.
enum class DeadlineMissPolicy : std::uint8_t {
    Replan,  ///< пересчитать график от текущего момента
    Fault,   ///< обесточить ось, снять доверие к позиции
};

/// Причина перехода в Released.
enum class ReleaseReason : std::uint8_t {
    HoldExpired,   ///< истёк конечный hold_timeout
    EmergencyStop, ///< аварийный останов
    Disable,       ///< ось выключена командой
    Explicit,      ///< команда Release
};

/// Причина перехода в Fault.
enum class FaultReason : std::uint8_t {
    DeadlineMiss,     ///< опоздание больше max_step_lateness при policy Fault
    ArmFailure,       ///< не удалось взвести alarm
    WatchdogTimeout,  ///< событие таймера не доставлено за watchdog-окно
    InvalidState,     ///< противоречивое внутреннее состояние
};

}  // namespace motion
