/**
 * @file stop_protocol.hpp
 * @brief Обязательный протокол останова пары осей.
 *
 * Подтверждение останова доказывается тикетом конкретной команды и эпохой
 * исполнителя, а не «нулевой скоростью» в снимке: снимок может быть прочитан
 * до применения команд. Ось, которая физически не может двигаться
 * (Fault/Disabled, обесточена, нулевая скорость), подтверждается без тикета.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "control/command_ticket.hpp"
#include "motion/axis_planner.hpp"
#include "motion/types.hpp"
#include "state/observation.hpp"

namespace control {

enum class StopReason : std::uint8_t {
    InputLoss,
    Disarm,
    SafetyStop,
    Handover,
    MotorFault,
    DeliveryLimit,
    Shutdown,
    Acknowledge
};

enum class StopMode : std::uint8_t { Smooth, Immediate };

/// Наблюдение одной оси при проверке останова.
struct StopObservation {
    state::Micros polled_us = 0;
    motion::AxisStatus status{};
};

class StopProtocol {
public:
    struct Config {
        /// Ожидание подтверждения до первой эскалации Immediate.
        state::Micros confirm_timeout_us = 1'500'000;
    };

    struct Start {
        StopReason reason = StopReason::InputLoss;
        StopMode mode = StopMode::Smooth;
        state::Micros now_us = 0;
        std::uint32_t baseline_epoch[2] = {0, 0};
    };

    struct Action {
        bool send[2] = {false, false};
        StopMode mode = StopMode::Smooth;
    };

    enum class Decision : std::uint8_t {
        Idle,       ///< протокол не запущен
        Pending,    ///< останов доставляется или ожидает подтверждения
        Confirmed,  ///< обе оси подтвердили
        Escalate,   ///< нужна эскалация Immediate
        Failed      ///< останов не подтверждён и эскалация исчерпана
    };

    void configure(const Config &config) noexcept;
    void begin(const Start &start) noexcept;
    void reset() noexcept;

    /// Что отправить на этом цикле. Для Immediate повторяет запрос при отказе.
    Action next() noexcept;
    void noteSent(motion::AxisId id, const Delivery &delivery) noexcept;
    void noteFailed(motion::AxisId id) noexcept;

    Decision check(const StopObservation (&obs)[2], state::Micros now_us) noexcept;

    /// Ровно одна эскалация: новый тикет, новый срок подтверждения.
    bool escalate(state::Micros now_us) noexcept;

    bool active() const noexcept { return active_; }
    bool confirmed() const noexcept { return confirmed_; }
    bool escalated() const noexcept { return escalated_; }
    StopReason reason() const noexcept { return reason_; }
    StopMode mode() const noexcept { return mode_; }
    state::Micros requestedUs() const noexcept { return requested_us_; }

    static bool axisAtRest(const motion::AxisStatus &status) noexcept;
    static bool axisCannotMove(const motion::AxisStatus &status) noexcept;

private:
    bool axisConfirmed(const StopObservation &obs, std::size_t axis) const noexcept;

    Config config_{};
    bool active_ = false;
    bool confirmed_ = false;
    bool escalated_ = false;
    StopReason reason_ = StopReason::InputLoss;
    StopMode mode_ = StopMode::Smooth;
    state::Micros requested_us_ = 0;
    bool sent_[2] = {false, false};
    CommandTicket ticket_[2]{};
    std::uint32_t baseline_epoch_[2] = {0, 0};
};

}  // namespace control
