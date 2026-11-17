/**
 * @file arming.cpp
 * @brief Реализация автомата разрешения движения.
 */

#include "control/arming.hpp"

namespace control {

void ArmingMachine::configure(const ArmingConfig &config) noexcept
{
    config_ = config;
    if (config_.delivery_failure_limit == 0U) {
        config_.delivery_failure_limit = 1;
    }
    reset();
}

void ArmingMachine::reset() noexcept
{
    phase_ = state::ControlPhase::Disarmed;
    latch_ = state::LatchCode::None;
    reason_ = StopReason::InputLoss;
}

void ArmingMachine::clearLatch() noexcept
{
    // Вызывается только после подтверждённого протокола восстановления.
    latch_ = state::LatchCode::None;
    phase_ = state::ControlPhase::Disarmed;
}

void ArmingMachine::restoreLatch(state::LatchCode latch) noexcept
{
    latch_ = latch;
    phase_ = state::ControlPhase::Disarmed;
}

void ArmingMachine::noteStopIssued(StopReason reason) noexcept
{
    reason_ = reason;
}

void ArmingMachine::beginStopping(StopReason reason) noexcept
{
    phase_ = state::ControlPhase::Stopping;
    reason_ = reason;
}

ArmingDecision ArmingMachine::step(const ArmingInputs &in) noexcept
{
    ArmingDecision out{};

    // 0. Выбор источника: ветвление, а не объединение. Пока владеет
    //    калибровка, её пригодность заменяет проверки нейтрали и входа.
    const bool calibration_selected = in.selected_source == state::SourceId::Calibration;
    const bool input_usable = calibration_selected ? in.calibration_input_ready
                                                   : in.input_usable;
    const bool neutral_confirmed = calibration_selected ? in.calibration_input_ready
                                                        : in.neutral_confirmed;

    // 1. Защёлка выставляется по аварийным входам и не снимается сама.
    if (latch_ == state::LatchCode::None) {
        if (in.motor_fault) {
            latch_ = state::LatchCode::MotorFault;
        } else if (in.axis_lost || in.rate_delivery_failed) {
            // Потеря готовности оси не должна оставлять вторую ось в движении.
            latch_ = state::LatchCode::DeliveryFailed;
        }
    }

    const bool blocked = latch_ != state::LatchCode::None;

    // 2. Приоритетные поводы останова: защёлка, safety stop, снятие permit,
    //    потеря ввода, передача владения.
    if (phase_ != state::ControlPhase::Stopping) {
        if (in.stop_requested) {
            beginStopping(StopReason::SafetyStop);
            out.request_stop = true;
            out.revoke_ownership = true;
        } else if (blocked) {
            beginStopping(in.motor_fault ? StopReason::MotorFault
                                         : StopReason::DeliveryLimit);
            out.request_stop = true;
            out.revoke_ownership = true;
        } else if (in.calibration_stop) {
            // Штатный останов для фиксации опорной точки: владение, permit и
            // удержание остаются, чтобы сессия могла продолжиться.
            beginStopping(StopReason::Calibration);
            out.request_stop = true;
        } else if (in.handover_pending && phase_ == state::ControlPhase::Active) {
            beginStopping(StopReason::Handover);
            out.request_stop = true;
            out.revoke_ownership = true;
        } else if (!in.permit && (phase_ == state::ControlPhase::Active ||
                                  phase_ == state::ControlPhase::Enabling ||
                                  phase_ == state::ControlPhase::WaitingForNeutral)) {
            beginStopping(StopReason::Disarm);
            out.request_stop = true;
            out.revoke_ownership = true;
        } else if (phase_ == state::ControlPhase::Active && !input_usable) {
            beginStopping(StopReason::InputLoss);
            out.request_stop = true;
            out.revoke_ownership = true;
        }
    }

    // 3. Переходы фазы.
    switch (phase_) {
        case state::ControlPhase::Disarmed:
            if (!blocked && in.permit) {
                phase_ = state::ControlPhase::WaitingForNeutral;
            }
            break;

        case state::ControlPhase::WaitingForNeutral:
            if (!in.permit || blocked) {
                phase_ = state::ControlPhase::Disarmed;
            } else if (neutral_confirmed) {
                phase_ = state::ControlPhase::Enabling;
                out.send_enable = true;
            }
            break;

        case state::ControlPhase::Enabling:
            if (in.enable_failed) {
                if (latch_ == state::LatchCode::None) {
                    latch_ = state::LatchCode::DeliveryFailed;
                }
                beginStopping(StopReason::DeliveryLimit);
                out.request_stop = true;
                out.revoke_ownership = true;
            } else if (in.axes_ready) {
                phase_ = state::ControlPhase::Active;
            }
            break;

        case state::ControlPhase::Active:
            break;

        case state::ControlPhase::Stopping:
            if (in.stop_requested || blocked) {
                out.request_stop = true;
            }
            if (in.stop_escalate) {
                out.request_stop = true;
            }
            if (in.stop_failed) {
                if (latch_ == state::LatchCode::None) {
                    latch_ = state::LatchCode::StopUnconfirmed;
                }
                phase_ = state::ControlPhase::Disarmed;
            } else if (in.stopped_confirmed) {
                if (reason_ == StopReason::Handover) {
                    if (in.handover_valid) {
                        phase_ = state::ControlPhase::Active;
                        out.handover_complete = true;
                    } else {
                        phase_ = state::ControlPhase::WaitingForNeutral;
                    }
                } else if (!in.permit || blocked) {
                    phase_ = state::ControlPhase::Disarmed;
                } else {
                    phase_ = state::ControlPhase::WaitingForNeutral;
                }
            }
            break;
    }

    out.phase = phase_;
    out.latch = latch_;
    out.stop_reason = reason_;
    out.allow_nonzero = in.permit && !blocked && input_usable &&
                        phase_ == state::ControlPhase::Active;
    return out;
}

}  // namespace control
