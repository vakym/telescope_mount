/**
 * @file intent.hpp
 * @brief Намерения источников и выбранная арбитражем уставка.
 *
 * Источник формирует намерение на пару осей целиком. Поля поколения в
 * Intent нет: поколение назначает арбитр и возвращает в ArbitratedSetpoint.
 */

#pragma once

#include <cstdint>

#include "state/channels.hpp"
#include "state/observation.hpp"

namespace control {

using state::DenyReason;
using state::Micros;
using state::SourceId;

/// Уставка одной оси.
struct AxisRate {
    bool valid = false;
    float half_steps_per_s = 0.0F;
};

/// Намерение источника: скорости, запрос владения и запрос останова.
struct Intent {
    SourceId source = SourceId::None;
    std::uint32_t session = 0;   ///< номер сессии источника
    std::uint32_t sequence = 0;  ///< порядок намерений внутри сессии
    Micros created_us = 0;
    Micros valid_until_us = 0;  ///< lease
    bool claim = false;
    bool release = false;
    bool stop = false;
    AxisRate axis[2]{};  ///< индекс = motion::AxisId
};

/// Итог арбитража на текущий цикл.
struct ArbitratedSetpoint {
    bool have_command = false;
    SourceId owner = SourceId::None;
    std::uint32_t owner_generation = 0;
    std::uint32_t session = 0;
    float rate[2] = {0.0F, 0.0F};
    bool stop = false;
    bool handover_pending = false;
    DenyReason deny = DenyReason::None;
};

}  // namespace control
