/**
 * @file calibration_command.hpp
 * @brief Разбор тела команд калибровки без библиотеки JSON.
 *
 * Поддерживается только плоский объект с известными ключами. Неизвестные
 * ключи игнорируются, отсутствие обязательного поля — ошибка. Числа
 * проверяются по диапазону; логика сессии остаётся в calibration_hal.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace web {

enum class CalibrationCommandKind : std::uint8_t {
    CreateSession,
    Move,
    Stop,
    ConfirmStart,
    ConfirmEnd,
    Apply,
    Close,
    Reset,
};

enum class CalibrationCommandResult : std::uint8_t {
    Ok,
    InvalidJson,
    MissingField,
    InvalidValue,
};

struct CalibrationCommand {
    std::uint32_t session_id = 0;
    std::uint32_t motion_generation = 0;
    std::uint32_t revolutions = 1;
    std::uint32_t repeats = 2;
    float rate = 0.0F;
    int axis = -1;       ///< 0 — азимут, 1 — высота
    int direction = -1;  ///< 0 — вперёд, 1 — назад
    bool acknowledge_low_repeatability = false;
};

/// Максимальный предел модуля скорости; сервис дополнительно ограничивает.
constexpr float kMaxCalibrationRate = 1000.0F;

CalibrationCommandResult parseCalibrationCommand(CalibrationCommandKind kind,
                                                 const char *body, std::size_t length,
                                                 CalibrationCommand &out) noexcept;

}  // namespace web
