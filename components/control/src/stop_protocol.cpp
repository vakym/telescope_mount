/**
 * @file stop_protocol.cpp
 * @brief Реализация протокола останова с подтверждением по тикету.
 */

#include "control/stop_protocol.hpp"

namespace control {
namespace {

bool moving(const motion::AxisStatus &status) noexcept
{
    return status.state == motion::AxisState::Accelerating ||
           status.state == motion::AxisState::Cruising ||
           status.state == motion::AxisState::Decelerating;
}

std::size_t indexOf(motion::AxisId id) noexcept
{
    return static_cast<std::size_t>(id);
}

}  // namespace

bool StopProtocol::axisAtRest(const motion::AxisStatus &status) noexcept
{
    return status.rate_steps_s == 0.0F && status.target_rate_steps_s == 0.0F &&
           !moving(status);
}

bool StopProtocol::axisCannotMove(const motion::AxisStatus &status) noexcept
{
    // Released сюда НЕ входит: обесточенная ось с enabled_ == true снова
    // примет команду скорости, поэтому покой надо доказывать тикетом.
    const bool refuses_commands = status.state == motion::AxisState::Fault ||
                                  status.state == motion::AxisState::Disabled;
    return refuses_commands && !status.energized && status.rate_steps_s == 0.0F;
}

void StopProtocol::configure(const Config &config) noexcept
{
    config_ = config;
    if (config_.confirm_timeout_us == 0U) {
        config_.confirm_timeout_us = 1'500'000;
    }
    reset();
}

void StopProtocol::reset() noexcept
{
    active_ = false;
    confirmed_ = false;
    escalated_ = false;
    reason_ = StopReason::InputLoss;
    mode_ = StopMode::Smooth;
    requested_us_ = 0;
    sent_[0] = false;
    sent_[1] = false;
    ticket_[0] = CommandTicket{};
    ticket_[1] = CommandTicket{};
    baseline_epoch_[0] = 0;
    baseline_epoch_[1] = 0;
}

void StopProtocol::begin(const Start &start) noexcept
{
    active_ = true;
    confirmed_ = false;
    escalated_ = (start.mode == StopMode::Immediate);
    reason_ = start.reason;
    mode_ = start.mode;
    requested_us_ = start.now_us;
    sent_[0] = false;
    sent_[1] = false;
    ticket_[0] = CommandTicket{};
    ticket_[1] = CommandTicket{};
    baseline_epoch_[0] = start.baseline_epoch[0];
    baseline_epoch_[1] = start.baseline_epoch[1];
}

StopProtocol::Action StopProtocol::next() noexcept
{
    Action action{};
    if (!active_ || confirmed_) {
        return action;
    }
    action.mode = mode_;
    for (std::size_t i = 0; i < 2; ++i) {
        action.send[i] = !sent_[i];
    }
    return action;
}

void StopProtocol::noteSent(motion::AxisId id, const Delivery &delivery) noexcept
{
    const std::size_t axis = indexOf(id);
    if (axis > 1) {
        return;
    }
    if (delivery.accepted && delivery.ticket.valid()) {
        sent_[axis] = true;
        ticket_[axis] = delivery.ticket;
    } else {
        sent_[axis] = false;  // повтор на следующем цикле
    }
}

void StopProtocol::noteFailed(motion::AxisId id) noexcept
{
    const std::size_t axis = indexOf(id);
    if (axis > 1) {
        return;
    }
    sent_[axis] = false;
}

bool StopProtocol::axisConfirmed(const StopObservation &obs,
                                 std::size_t axis) const noexcept
{
    if (axis > 1) {
        return false;
    }
    // Ось, которая физически не может двигаться, подтверждается без тикета.
    if (axisCannotMove(obs.status)) {
        return true;
    }
    if (!sent_[axis] || !ticket_[axis].valid()) {
        return false;
    }
    if (obs.polled_us <= requested_us_) {
        return false;  // снимок прочитан до запроса останова
    }
    if (obs.status.command_epoch != ticket_[axis].epoch) {
        return false;
    }
    if (obs.status.last_applied_command_id != ticket_[axis].id) {
        return false;
    }
    return axisAtRest(obs.status);
}

StopProtocol::Decision StopProtocol::check(const StopObservation (&obs)[2],
                                           state::Micros now_us) noexcept
{
    if (!active_) {
        return Decision::Idle;
    }
    if (confirmed_) {
        return Decision::Confirmed;
    }
    if (axisConfirmed(obs[0], 0) && axisConfirmed(obs[1], 1)) {
        confirmed_ = true;
        return Decision::Confirmed;
    }
    if (now_us >= requested_us_ + config_.confirm_timeout_us) {
        return escalated_ ? Decision::Failed : Decision::Escalate;
    }
    return Decision::Pending;
}

bool StopProtocol::escalate(state::Micros now_us) noexcept
{
    if (!active_ || escalated_) {
        return false;
    }
    escalated_ = true;
    mode_ = StopMode::Immediate;
    sent_[0] = false;
    sent_[1] = false;
    ticket_[0] = CommandTicket{};
    ticket_[1] = CommandTicket{};
    requested_us_ = now_us;  // новый срок подтверждения
    return true;
}

}  // namespace control
