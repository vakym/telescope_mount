/**
 * @file joystick_config.hpp
 * @brief Базовые типы обработки положений джойстика.
 *
 * Компонент input — чистая логика C++17 без зависимости от ESP-IDF.
 * Логические оси X и Y соответствуют VRx и VRy; это не оси монтировки.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace input {

/// Логические оси стика: X — VRx, Y — VRy.
enum class StickAxis : std::uint8_t { X = 0, Y = 1, Count = 2 };

constexpr std::size_t kAxisCount = static_cast<std::size_t>(StickAxis::Count);

/// Один сырой отсчёт АЦП; valid = false означает ошибку чтения.
struct RawSample {
    std::uint16_t counts[kAxisCount] = {0, 0};
    bool valid = false;
};

/// Калибровка одной оси стика.
struct AxisCalibration {
    float center = 2048.0F;  ///< среднее положение, отсчёты АЦП
    float min = 0.0F;        ///< нижний край, отсчёты АЦП
    float max = 4095.0F;     ///< верхний край, отсчёты АЦП
    bool invert = false;     ///< инверсия знака оси
};

/// Конфигурация обработчика положений.
struct JoystickConfig {
    AxisCalibration axis[kAxisCount]{};

    float dead_zone = 0.12F;       ///< вход в активную зону, доля модуля
    float dead_zone_exit = 0.10F;  ///< возврат в нейтраль, доля модуля
    float filter_alpha = 0.25F;    ///< EMA, (0, 1]
};

/// Состояние стика после обработки.
struct StickState {
    bool neutral = true;
    float filtered[kAxisCount] = {0.0F, 0.0F};   ///< отсчёты после EMA
    float normalized[kAxisCount] = {0.0F, 0.0F}; ///< -1..1 с учётом invert
    /// Ограниченная единицей величина отклонения. Это не математический
    /// модуль вектора normalized: компоненты нормированы независимо, а их
    /// длина может достигать sqrt(2).
    float magnitude = 0.0F;
};

/**
 * @brief Проверяет конфигурацию перед запуском.
 *
 * Невалидная конфигурация — ошибка инициализации, а не повод молча
 * скорректировать значения.
 */
bool isValid(const JoystickConfig &config) noexcept;

}  // namespace input
