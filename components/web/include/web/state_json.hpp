/**
 * @file state_json.hpp
 * @brief Контракт и ограниченная сериализация снимка состояния для HTTP.
 *
 * Документ собирается в буфер фиксированного размера без динамических строк
 * и без библиотеки JSON. Единицы: позиция — полушаги, скорость — полушаги в
 * секунду. Программная позиция не называется углом.
 *
 * Правила значений (совпадают с планом и интерфейсом):
 *  - блок никогда не публиковался или инвалидирован: значения `null`;
 *  - блок опубликован и устарел: значения сохраняются, `health` = "stale"
 *    и фактический `age_ms`;
 *  - непригодное отдельное число (NaN/Inf/вне диапазона): `null` только в
 *    этом поле, остальной блок сохраняется.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "state/observation.hpp"

namespace web {

/// Верхняя граница документа. Худший случай покрыт host-тестом.
constexpr std::size_t kMaxStateJsonBytes = 2048;

/// Версия контракта.
constexpr std::uint32_t kSchemaVersion = 1;

/// Предел модуля скорости, полушаги/с. Выше — значение не публикуется.
constexpr float kMaxRateAbsHalfStepsS = 10000.0F;

/// Пороги свежести по времени источника.
struct SerializeOptions {
    state::Micros axis_stale_us = 500000;
    state::Micros joystick_stale_us = 500000;
    /// Возраст контура считается по polled_us: measured_us у него всегда kNever.
    state::Micros control_stale_us = 500000;
};

enum class SerializeResult : std::uint8_t { Ok, Overflow };

/**
 * @brief Записывает документ состояния.
 *
 * @param snapshot согласованная копия; now берётся из generated_us
 * @param options пороги свежести
 * @param out буфер вывода
 * @param capacity полная ёмкость буфера, включая завершающий '\0'
 * @param written длина документа без завершающего нуля (0 при Overflow)
 *
 * При Overflow буфер не содержит частично валидного документа: первый байт
 * обнуляется, а вызывающий обязан ответить ошибкой, а не отправлять данные.
 */
SerializeResult stateJson(const state::SystemSnapshot &snapshot,
                          const SerializeOptions &options, char *out,
                          std::size_t capacity,
                          std::size_t *written) noexcept;

}  // namespace web
