/**
 * @file control_fakes.hpp
 * @brief Тестовый порт исполнения: тикеты, эпоха, FIFO и задержка применения.
 *
 * Позволяет воспроизвести сценарии, которые нельзя проверить на реальном
 * железе в host-тесте: «ось стоит, команда ещё в очереди», «emergency отправлен,
 * но не обработан», частичный успех пары.
 */

#pragma once

#include <array>
#include <cstdint>

#include "control/actuator.hpp"
#include "motion/axis_planner.hpp"

namespace fakes {

class FakeActuator final : public control::AxisActuator {
public:
    static constexpr std::size_t kAxisCount = 2;

    struct AxisState {
        motion::AxisStatus status{};
        float target_rate = 0.0f;
        bool fail_rate = false;
        bool fail_enable = false;
        bool fail_stop = false;
        bool fail_reset = false;
        bool reject_rates = false;      ///< ось выключена: команда отклоняется
        int enable_calls = 0;
        int rate_calls = 0;
        int stop_calls = 0;
        int immediate_calls = 0;
        int reset_calls = 0;
    };

    FakeActuator()
    {
        for (auto &axis : axes_) {
            axis.status.state = motion::AxisState::Idle;
            axis.status.command_epoch = kEpoch;
        }
    }

    std::uint64_t next_id = 1;
    std::uint32_t epoch = kEpoch;
    static constexpr std::uint32_t kEpoch = 7;

    /// Команда принята, но ещё не применена (сценарий «старый покой»).
    void queueRateWithoutApplying(std::size_t axis, float rate)
    {
        pending_[axis] = true;
        pending_rate_[axis] = rate;
    }

    /// Применить отложенную команду: имитация пробуждения задачи оси.
    void applyPending(std::size_t axis)
    {
        if (!pending_[axis]) {
            return;
        }
        pending_[axis] = false;
        if (axes_[axis].reject_rates) {
            return;
        }
        axes_[axis].status.last_applied_command_id = pending_id_[axis];
        axes_[axis].status.target_rate_steps_s = pending_rate_[axis];
        axes_[axis].status.rate_steps_s = pending_rate_[axis];
        axes_[axis].status.state = pending_rate_[axis] == 0.0f
                                       ? motion::AxisState::Released
                                       : motion::AxisState::Cruising;
    }

    AxisState &axis(std::size_t index) { return axes_[index]; }
    const AxisState &axis(std::size_t index) const { return axes_[index]; }

    control::Delivery enable(motion::AxisId id) override
    {
        AxisState &state = axes_[static_cast<std::size_t>(id)];
        ++state.enable_calls;
        if (state.fail_enable) {
            return control::Delivery{};
        }
        state.status.state = motion::AxisState::Idle;
        return ok();
    }

    control::Delivery setRate(motion::AxisId id, float rate) override
    {
        const std::size_t index = static_cast<std::size_t>(id);
        AxisState &state = axes_[index];
        ++state.rate_calls;
        if (state.fail_rate) {
            return control::Delivery{};
        }
        const std::uint64_t ticket = next_id++;
        pending_[index] = true;
        pending_rate_[index] = rate;
        pending_id_[index] = ticket;
        return accepted(ticket);
    }

    control::Delivery stopSmooth(motion::AxisId id) override
    {
        const std::size_t index = static_cast<std::size_t>(id);
        AxisState &state = axes_[index];
        ++state.stop_calls;
        if (state.fail_stop) {
            return control::Delivery{};
        }
        const std::uint64_t ticket = next_id++;
        pending_[index] = true;
        pending_rate_[index] = 0.0f;
        pending_id_[index] = ticket;
        return accepted(ticket);
    }

    control::Delivery stopImmediate(motion::AxisId id) override
    {
        const std::size_t index = static_cast<std::size_t>(id);
        AxisState &state = axes_[index];
        ++state.immediate_calls;
        if (state.fail_stop) {
            return control::Delivery{};
        }
        const std::uint64_t ticket = next_id++;
        pending_[index] = false;
        state.status.last_applied_command_id = ticket;
        state.status.target_rate_steps_s = 0.0f;
        state.status.rate_steps_s = 0.0f;
        state.status.energized = false;
        state.status.state = motion::AxisState::Released;
        return accepted(ticket);
    }

    control::Delivery resetFault(motion::AxisId id) override
    {
        const std::size_t index = static_cast<std::size_t>(id);
        AxisState &state = axes_[index];
        ++state.reset_calls;
        if (state.fail_reset) {
            return control::Delivery{};
        }
        const std::uint64_t ticket = next_id++;
        state.status.has_fault = false;
        state.status.state = motion::AxisState::Disabled;
        state.status.energized = false;
        state.status.last_applied_command_id = ticket;
        return accepted(ticket);
    }

    motion::AxisStatus status(motion::AxisId id) const override
    {
        return axes_[static_cast<std::size_t>(id)].status;
    }

private:
    static control::Delivery ok()
    {
        control::Delivery delivery{};
        delivery.accepted = true;
        delivery.notified = true;
        return delivery;
    }

    control::Delivery accepted(std::uint64_t ticket)
    {
        control::Delivery delivery{};
        delivery.accepted = true;
        delivery.notified = true;
        delivery.ticket.id = ticket;
        delivery.ticket.epoch = epoch;
        return delivery;
    }

    std::array<AxisState, kAxisCount> axes_{};
    bool pending_[kAxisCount] = {false, false};
    float pending_rate_[kAxisCount] = {0.0f, 0.0f};
    std::uint64_t pending_id_[kAxisCount] = {0, 0};
};

}  // namespace fakes
