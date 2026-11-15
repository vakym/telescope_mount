/**
 * @file rate_gate.hpp
 * @brief Дедупликация уставок: не отправлять одинаковые и малозначимые.
 *
 * Хранится последняя УСПЕШНО принятая уставка и признак неудачной попытки.
 * Повтор после отказа отправляет актуальное желаемое значение, а не прежнее;
 * нуль и останов вытесняют ожидающую ненулевую уставку. Пара осей не
 * синхронизируется: успешная ось не переотправляется ради неуспешной.
 */

#pragma once

#include "motion/types.hpp"
#include "state/channels.hpp"

namespace control {

class RateGate {
public:
    struct Config {
        /// Минимальное изменение уставки, которое стоит отправлять.
        float deadband = 8.0F;
    };

    struct Decision {
        bool send = false;
        float value = 0.0F;
    };

    void configure(const Config &config) noexcept;
    void reset() noexcept;

    /// desired — актуальная уставка текущего поколения; 0 при останове.
    Decision evaluate(motion::AxisId id, float desired,
                      std::uint32_t generation) noexcept;
    void noteEnqueued(motion::AxisId id, float value,
                      std::uint32_t generation) noexcept;
    void noteFailed(motion::AxisId id) noexcept;

    bool hasEnqueuedNonZero(motion::AxisId id) const noexcept;
    float lastEnqueued(motion::AxisId id) const noexcept;
    bool pending(motion::AxisId id) const noexcept;

private:
    struct Slot {
        bool known = false;
        bool pending = false;
        float value = 0.0F;
    };

    static std::size_t indexOf(motion::AxisId id) noexcept;

    Config config_{};
    Slot slots_[2]{};
    std::uint32_t generation_ = 0;
};

}  // namespace control
