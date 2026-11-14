/**
 * @file coil_pattern.hpp
 * @brief Фазовые таблицы ULN2003.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "motion/types.hpp"

namespace motor_hal {

/// Состояние четырёх линий: бит 0 — IN1, бит 3 — IN4.
using PhaseMask = std::uint8_t;

constexpr std::size_t patternLength(motion::StepPattern pattern) noexcept
{
    return pattern == motion::StepPattern::Half8 ? 8U : 4U;
}

constexpr PhaseMask halfStep8[8] = {
    0b0001,  // IN1
    0b0011,  // IN1 IN2
    0b0010,  // IN2
    0b0110,  // IN2 IN3
    0b0100,  // IN3
    0b1100,  // IN3 IN4
    0b1000,  // IN4
    0b1001,  // IN4 IN1
};

constexpr PhaseMask fullStepDouble[4] = {
    0b0011,  // IN1 IN2
    0b0110,  // IN2 IN3
    0b1100,  // IN3 IN4
    0b1001,  // IN4 IN1
};

constexpr PhaseMask fullStepWave[4] = {
    0b0001,  // IN1
    0b0010,  // IN2
    0b0100,  // IN3
    0b1000,  // IN4
};

inline const PhaseMask *tableFor(motion::StepPattern pattern) noexcept
{
    switch (pattern) {
    case motion::StepPattern::Half8:
        return halfStep8;
    case motion::StepPattern::Full4Double:
        return fullStepDouble;
    case motion::StepPattern::Full4Wave:
        return fullStepWave;
    }
    return halfStep8;
}

}  // namespace motor_hal
