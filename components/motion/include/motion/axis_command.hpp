/**
 * @file axis_command.hpp
 * @brief Команды, которые исполняет задача оси.
 */

#pragma once

#include <cstdint>

#include "motion/axis_config.hpp"
#include "motion/types.hpp"

namespace motion {

/// Вид команды.
enum class CommandKind : std::uint8_t {
    Enable,
    Disable,
    SetTargetRate,  ///< value = полушагов/с, знак задаёт направление
    MoveSteps,      ///< steps = относительный ход в полушагах, кратный delta
    StopSmooth,
    StopImmediate,
    Release,        ///< обесточить, позиция сохраняется
    Energize,       ///< снова подать фазы в текущей позиции
    SetHoldPolicy,
    ResetFault,     ///< сброс ошибки, движение разрешается после Enable
};

/// Унифицированная команда оси.
struct AxisCommand {
    CommandKind kind = CommandKind::Enable;
    float value = 0.0F;
    std::int64_t steps = 0;
    HoldPolicy hold{};
    /// Идентификатор команды. Поле-результат: назначается в единственной
    /// точке отправки (AxisMotor::post) и не читается ею на входе.
    /// 0 означает «идентификатор ещё не назначен».
    std::uint64_t id = 0;

    static AxisCommand enable() noexcept;
    static AxisCommand disable() noexcept;
    static AxisCommand setTargetRate(float half_steps_per_s) noexcept;
    static AxisCommand moveSteps(std::int64_t half_steps) noexcept;
    static AxisCommand stop(StopMode mode) noexcept;
    static AxisCommand release() noexcept;
    static AxisCommand energize() noexcept;
    static AxisCommand setHoldPolicy(const HoldPolicy &policy) noexcept;
    static AxisCommand resetFault() noexcept;
};

}  // namespace motion
