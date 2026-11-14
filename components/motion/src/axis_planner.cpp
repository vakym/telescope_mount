/**
 * @file axis_planner.cpp
 * @brief Профиль разгона/торможения и двухфазный контракт исполнения.
 *
 * Математика участка: dt = 2*ds/(v_begin + v_end), где ds — полушаги одного
 * переключения (1 для Half8, 2 для full-step). Скорости на границах:
 * v_end^2 = v_begin^2 + 2*a*ds при разгоне и
 * v_end^2 = max(0, v_begin^2 - 2*d*ds) при торможении.
 */

#include "motion/axis_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace motion {
namespace {

constexpr float kRateEpsilon = 1e-3F;
constexpr double kUsPerSecond = 1'000'000.0;

std::int32_t signOf(std::int64_t value) noexcept
{
    if (value > 0) {
        return 1;
    }
    if (value < 0) {
        return -1;
    }
    return 0;
}

bool sameHoldPolicy(const HoldPolicy &a, const HoldPolicy &b) noexcept
{
    return a.hold_enabled == b.hold_enabled && a.hold_timeout_ms == b.hold_timeout_ms;
}

}  // namespace

bool AxisPlanner::configure(const AxisConfig &config) noexcept
{
    if (!isValid(config)) {
        return false;
    }

    config_ = config;
    configured_ = true;
    enabled_ = false;
    fault_latched_ = false;
    state_ = AxisState::Disabled;
    phase_ = Phase::Idle;

    position_mode_ = false;
    target_position_ = 0;
    position_ = 0;

    v_ = 0.0F;
    dir_ = Direction::Forward;
    pause_dir_ = Direction::Forward;
    target_rate_ = 0.0F;

    position_trusted_ = true;
    intent_rev_ = 0;
    pending_ = Pending{};
    status_ = AxisStatus{};
    updateStatus();
    return true;
}

bool AxisPlanner::reject() noexcept
{
    ++status_.rejected_commands;
    return false;
}

bool AxisPlanner::apply(const AxisCommand &cmd) noexcept
{
    if (!configured_) {
        return reject();
    }

    switch (cmd.kind) {
    case CommandKind::Enable:
        if (fault_latched_) {
            return reject();
        }
        if (enabled_) {
            return true;
        }
        enabled_ = true;
        if (v_ <= kRateEpsilon && phase_ == Phase::Idle) {
            state_ = AxisState::Idle;
        }
        ++intent_rev_;
        updateStatus();
        return true;

    case CommandKind::Disable:
        if (!enabled_ && state_ == AxisState::Disabled) {
            return true;
        }
        ++intent_rev_;
        forceReleased(ReleaseReason::Disable);
        return true;

    case CommandKind::SetTargetRate: {
        if (!enabled_ || fault_latched_) {
            return reject();
        }
        if (!std::isfinite(cmd.value)) {
            return reject();
        }
        float target = cmd.value;
        if (target > config_.max_rate_steps_s) {
            target = config_.max_rate_steps_s;
        }
        if (target < -config_.max_rate_steps_s) {
            target = -config_.max_rate_steps_s;
        }
        if (!position_mode_ && std::fabs(target - target_rate_) <= kRateEpsilon) {
            return true;  // идемпотентная уставка
        }
        target_rate_ = target;
        position_mode_ = false;
        ++intent_rev_;
        updateStatus();
        return true;
    }

    case CommandKind::MoveSteps: {
        if (!enabled_ || fault_latched_) {
            return reject();
        }
        const std::int32_t ds = deltaHalfSteps(config_.pattern);
        if (cmd.steps == 0) {
            if (!position_mode_ && std::fabs(target_rate_) <= kRateEpsilon &&
                phase_ == Phase::Idle && v_ <= kRateEpsilon) {
                return true;
            }
            target_rate_ = 0.0F;
            position_mode_ = false;
            ++intent_rev_;
            updateStatus();
            return true;
        }
        if (cmd.steps % ds != 0) {
            return reject();  // нечётный ход в full-step не округляется
        }
        if ((cmd.steps > 0 && position_ > std::numeric_limits<std::int64_t>::max() - cmd.steps) ||
            (cmd.steps < 0 && position_ < std::numeric_limits<std::int64_t>::min() - cmd.steps)) {
            return reject();  // переполнение целевой позиции
        }
        position_mode_ = true;
        target_position_ = position_ + cmd.steps;
        target_rate_ = 0.0F;
        if (phase_ != Phase::Pause) {
            phase_ = Phase::Move;
        }
        ++intent_rev_;
        updateStatus();
        return true;
    }

    case CommandKind::StopSmooth:
        if (!position_mode_ && std::fabs(target_rate_) <= kRateEpsilon) {
            return true;
        }
        target_rate_ = 0.0F;
        position_mode_ = false;
        ++intent_rev_;
        updateStatus();
        return true;

    case CommandKind::StopImmediate:
        ++intent_rev_;
        forceReleased(ReleaseReason::EmergencyStop);
        return true;

    case CommandKind::Release:
        ++intent_rev_;
        forceReleased(ReleaseReason::Explicit);
        return true;

    case CommandKind::Energize:
        if (v_ > kRateEpsilon || pending_.valid || state_ == AxisState::Fault ||
            state_ == AxisState::Disabled) {
            return reject();
        }
        state_ = AxisState::Holding;
        updateStatus();
        return true;

    case CommandKind::SetHoldPolicy:
        if (sameHoldPolicy(config_.hold, cmd.hold)) {
            return true;
        }
        config_.hold = cmd.hold;
        ++intent_rev_;
        updateStatus();
        return true;

    case CommandKind::ResetFault:
        if (!fault_latched_) {
            return true;
        }
        resetFault();
        return true;
    }

    return reject();
}

bool AxisPlanner::wantsStep() const noexcept
{
    if (!configured_ || !enabled_ || fault_latched_) {
        return false;
    }
    if (phase_ == Phase::Pause) {
        return config_.reversal_pause_us > 0;
    }
    if (position_mode_) {
        return target_position_ != position_;
    }
    if (std::fabs(target_rate_) > kRateEpsilon) {
        return true;
    }
    return v_ > kRateEpsilon;
}

bool AxisPlanner::prepareNextStep(StepPlan &out) noexcept
{
    if (!configured_ || !enabled_ || fault_latched_) {
        return false;
    }

    if (pending_.valid) {
        out.direction = pending_.direction;
        out.interval_us = pending_.interval_us;
        out.is_step = !pending_.is_pause;
        return true;
    }

    const std::int32_t ds = deltaHalfSteps(config_.pattern);

    std::int64_t remaining_signed = 0;
    std::int64_t remaining = 0;
    if (position_mode_) {
        remaining_signed = target_position_ - position_;
        remaining = remaining_signed < 0 ? -remaining_signed : remaining_signed;
    }

    const float target_abs = std::fabs(target_rate_);
    const bool moving = v_ > kRateEpsilon;

    std::int32_t want = 0;
    if (position_mode_) {
        want = signOf(remaining_signed);
    } else if (target_abs > kRateEpsilon) {
        want = target_rate_ > 0.0F ? 1 : -1;
    }

    if (phase_ == Phase::Pause) {
        if (config_.reversal_pause_us == 0) {
            phase_ = Phase::Move;
        } else {
            Pending pause;
            pause.valid = true;
            pause.is_pause = true;
            pause.direction = pause_dir_;
            pause.interval_us = config_.reversal_pause_us;
            pending_ = pause;
            out.direction = pause.direction;
            out.interval_us = pause.interval_us;
            out.is_step = false;
            return true;
        }
    }

    if (position_mode_ && remaining == 0 && !moving) {
        finishMotion();
        return false;
    }
    if (!moving && want == 0) {
        finishMotion();
        return false;
    }

    const Direction want_dir = want > 0 ? Direction::Forward : Direction::Backward;
    const bool direction_conflict = moving && want != 0 && dir_ != want_dir;
    const bool braking = moving && (want == 0 || direction_conflict);

    double v_begin = static_cast<double>(v_);
    if (!moving) {
        double start = static_cast<double>(config_.start_rate_steps_s);
        if (!position_mode_ && target_abs > kRateEpsilon) {
            start = std::min(start, static_cast<double>(target_abs));
        }
        v_begin = start;
    }

    double v_end = 0.0;
    bool reverse_after = false;

    if (braking) {
        const double decel = static_cast<double>(config_.decel_steps_s2);
        const double sq = v_begin * v_begin - 2.0 * decel * static_cast<double>(ds);
        v_end = sq > 0.0 ? std::sqrt(sq) : 0.0;
        reverse_after = (v_end <= static_cast<double>(kRateEpsilon)) && (want != 0);
    } else {
        double limit = static_cast<double>(config_.max_rate_steps_s);
        if (!position_mode_ && target_abs > kRateEpsilon) {
            limit = std::min(limit, static_cast<double>(target_abs));
        }
        if (!moving) {
            v_end = std::min(v_begin, limit);  // первый шаг без разгона внутри участка
        } else {
            const double accel = static_cast<double>(config_.accel_steps_s2);
            const double sq = v_begin * v_begin + 2.0 * accel * static_cast<double>(ds);
            v_end = std::min(std::sqrt(sq), limit);
        }
    }

    if (v_end > static_cast<double>(config_.max_rate_steps_s)) {
        v_end = static_cast<double>(config_.max_rate_steps_s);
    }

    if (position_mode_) {
        const std::int64_t remaining_after = remaining - ds;
        const double cap = remaining_after > 0
                               ? std::sqrt(2.0 * static_cast<double>(config_.decel_steps_s2) *
                                           static_cast<double>(remaining_after))
                               : 0.0;
        const double decel = static_cast<double>(config_.decel_steps_s2);
        if (!moving) {
            // Старт: если цель ближе одного-двух событий, выходим на нулевую
            // скорость к цели — явные начальные условия одиночного события.
            if (v_end > cap) {
                v_end = cap;
            }
        } else {
            // Нельзя тормозить быстрее заданного decel: если цель ближе
            // тормозного пути, ось проходит её и возвращается штатным
            // реверсом после полной остановки.
            const double sq = v_begin * v_begin - 2.0 * decel * static_cast<double>(ds);
            const double v_min = sq > 0.0 ? std::sqrt(sq) : 0.0;
            // Последнее событие завершается нулевой скоростью только на
            // «тактирующей» скорости: до него скорость уже ограничена cap
            // предыдущих событий. На высокой скорости остановка в цели
            // невозможна, ось проходит её и возвращается реверсом.
            const double slow_enough = 2.0 * static_cast<double>(config_.start_rate_steps_s);
            const bool final_event = (remaining_after == 0) && (v_begin <= slow_enough);
            const double allowed = final_event ? 0.0 : std::max(cap, v_min);
            if (v_end > allowed) {
                v_end = allowed;
            }
        }
    }

    const double v_sum = v_begin + v_end;
    if (v_sum <= static_cast<double>(kRateEpsilon)) {
        finishMotion();
        return false;
    }

    double interval = 2.0 * static_cast<double>(ds) * kUsPerSecond / v_sum;
    if (interval < static_cast<double>(config_.min_phase_interval_us)) {
        interval = static_cast<double>(config_.min_phase_interval_us);
    }
    constexpr double kMaxInterval = static_cast<double>(std::numeric_limits<std::uint32_t>::max());
    if (interval > kMaxInterval) {
        interval = kMaxInterval;
    }

    Pending next;
    next.valid = true;
    next.is_pause = false;
    next.reverse_after = reverse_after;
    next.direction = moving ? dir_ : want_dir;
    next.interval_us = static_cast<std::uint32_t>(std::llround(interval));
    next.rate_before = static_cast<float>(v_begin);
    next.rate_after = static_cast<float>(v_end);
    pending_ = next;

    out.direction = next.direction;
    out.interval_us = next.interval_us;
    out.is_step = true;
    return true;
}

void AxisPlanner::commitPending() noexcept
{
    if (!pending_.valid) {
        return;
    }

    const Pending committed = pending_;
    pending_ = Pending{};

    if (committed.is_pause) {
        phase_ = Phase::Move;
        dir_ = committed.direction;
        updateStatus();
        return;
    }

    const std::int32_t ds = deltaHalfSteps(config_.pattern);
    position_ += (committed.direction == Direction::Forward) ? ds : -ds;
    dir_ = committed.direction;
    v_ = committed.rate_after;

    if (v_ <= kRateEpsilon) {
        v_ = 0.0F;
        const std::int64_t remaining = position_mode_ ? target_position_ - position_ : 0;
        const bool reversing = committed.reverse_after || remaining != 0;
        if (reversing && config_.reversal_pause_us > 0) {
            phase_ = Phase::Pause;
            if (remaining > 0) {
                pause_dir_ = Direction::Forward;
            } else if (remaining < 0) {
                pause_dir_ = Direction::Backward;
            } else {
                pause_dir_ = committed.direction == Direction::Forward ? Direction::Backward
                                                                       : Direction::Forward;
            }
        }
    }

    if (position_mode_ && target_position_ == position_ && v_ <= kRateEpsilon) {
        finishMotion();
        return;
    }

    updateState(committed.rate_before, committed.rate_after);
}

void AxisPlanner::cancelPendingStep() noexcept
{
    pending_ = Pending{};
}

void AxisPlanner::forceReleased(ReleaseReason reason) noexcept
{
    pending_ = Pending{};
    v_ = 0.0F;
    target_rate_ = 0.0F;
    position_mode_ = false;
    phase_ = Phase::Idle;

    if (reason == ReleaseReason::Disable) {
        enabled_ = false;
        state_ = AxisState::Disabled;
        position_trusted_ = false;
    } else {
        state_ = AxisState::Released;
        if (reason == ReleaseReason::EmergencyStop) {
            position_trusted_ = false;
        }
    }
    updateStatus();
}

void AxisPlanner::enterFault(FaultReason reason) noexcept
{
    (void)reason;
    pending_ = Pending{};
    v_ = 0.0F;
    target_rate_ = 0.0F;
    position_mode_ = false;
    phase_ = Phase::Idle;
    fault_latched_ = true;
    state_ = AxisState::Fault;
    position_trusted_ = false;
    ++intent_rev_;
    updateStatus();
}

void AxisPlanner::resetFault() noexcept
{
    if (!fault_latched_) {
        return;
    }
    fault_latched_ = false;
    enabled_ = false;
    pending_ = Pending{};
    state_ = AxisState::Disabled;
    phase_ = Phase::Idle;
    ++intent_rev_;
    updateStatus();
}

std::uint32_t AxisPlanner::planKey() const noexcept
{
    std::uint32_t key = static_cast<std::uint32_t>(intent_rev_ & 0x00FFFFFFU) << 4U;
    if (pending_.valid) {
        key |= 1U;
    }
    if (pending_.is_pause) {
        key |= 2U;
    }
    if (phase_ == Phase::Pause) {
        key |= 4U;
    }
    return key;
}

void AxisPlanner::finishMotion() noexcept
{
    pending_ = Pending{};
    v_ = 0.0F;
    target_rate_ = 0.0F;
    position_mode_ = false;
    phase_ = Phase::Idle;
    state_ = config_.hold.hold_enabled ? AxisState::Holding : AxisState::Released;
    updateStatus();
}

void AxisPlanner::updateState(float rate_before, float rate_after) noexcept
{
    if (v_ <= kRateEpsilon) {
        state_ = config_.hold.hold_enabled ? AxisState::Holding : AxisState::Released;
    } else if (rate_after < rate_before - kRateEpsilon) {
        state_ = AxisState::Decelerating;
    } else if (rate_after > rate_before + kRateEpsilon) {
        state_ = AxisState::Accelerating;
    } else {
        state_ = AxisState::Cruising;
    }
    updateStatus();
}

void AxisPlanner::updateStatus() noexcept
{
    status_.state = state_;
    status_.position_step = position_;
    status_.rate_steps_s = dir_ == Direction::Forward ? v_ : -v_;
    status_.target_rate_steps_s = target_rate_;
    status_.homed = false;
    status_.position_trusted = position_trusted_;
}

}  // namespace motion
