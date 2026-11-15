/**
 * @file arbiter.cpp
 * @brief Реализация арбитража владения парой осей.
 */

#include "control/arbiter.hpp"

namespace control {

bool Arbiter::validSource(SourceId source) noexcept
{
    return source != SourceId::None && source != SourceId::Count;
}

std::size_t Arbiter::indexOf(SourceId source) noexcept
{
    return static_cast<std::size_t>(source);
}

int Arbiter::priorityRank(SourceId source) noexcept
{
    switch (source) {
        case SourceId::Joystick:
            return 3;
        case SourceId::Browser:
            return 2;
        case SourceId::Automation:
            return 1;
        case SourceId::None:
        case SourceId::Count:
            break;
    }
    return 0;
}

void Arbiter::configure(const Config &config) noexcept
{
    config_ = config;
    if (config_.default_lease_us == 0U) {
        config_.default_lease_us = 200'000;
    }
    reset();
}

void Arbiter::reset() noexcept
{
    for (std::size_t i = 0; i < state::kSourceCount; ++i) {
        slots_[i] = Slot{};
    }
    owner_ = SourceId::None;
    generation_ = 0;
    pending_claim_ = SourceId::None;
    handover_pending_ = false;
    owner_expired_ = false;
}

void Arbiter::applyOwnership(SourceId source, bool claim, bool release,
                             std::uint32_t session, state::Micros now_us,
                             DenyReason &out_deny) noexcept
{
    (void)now_us;
    out_deny = DenyReason::None;
    if (!validSource(source)) {
        out_deny = DenyReason::InvalidConfig;
        return;
    }
    const std::size_t index = indexOf(source);

    if (release) {
        slots_[index].occupied = false;
        slots_[index].axis[0] = AxisRate{};
        slots_[index].axis[1] = AxisRate{};
        if (owner_ == source) {
            owner_ = SourceId::None;
            ++generation_;
        }
        if (pending_claim_ == source) {
            cancelHandover();
        }
        return;
    }

    if (!claim) {
        if (owner_ != source) {
            out_deny = DenyReason::NotOwner;
        }
        return;
    }

    // Претензия на владение.
    if (slots_[index].session == 0U) {
        slots_[index].session = session;
    }
    if (handover_pending_) {
        if (source != pending_claim_) {
            out_deny = DenyReason::StopInProgress;
        }
        return;
    }
    if (owner_ == SourceId::None) {
        owner_ = source;
        ++generation_;
        return;
    }
    if (owner_ == source) {
        return;
    }
    if (priorityRank(source) > priorityRank(owner_)) {
        // Слот прежнего владельца очищается: самопроизвольного возврата нет.
        const std::size_t previous = indexOf(owner_);
        slots_[previous].occupied = false;
        slots_[previous].axis[0] = AxisRate{};
        slots_[previous].axis[1] = AxisRate{};
        owner_ = SourceId::None;
        ++generation_;
        pending_claim_ = source;
        handover_pending_ = true;
        return;
    }
    out_deny = DenyReason::NotOwner;
}

void Arbiter::applyRates(SourceId source, const AxisRate (&axis)[2],
                         std::uint32_t session, state::Micros created_us,
                         state::Micros valid_until_us) noexcept
{
    (void)created_us;
    if (!validSource(source) || session == 0U) {
        return;
    }
    Slot &slot = slots_[indexOf(source)];
    if (slot.occupied && slot.session > session) {
        return;  // старое сообщение не вытесняет новое
    }
    slot.session = session;
    slot.axis[0] = axis[0];
    slot.axis[1] = axis[1];
    slot.valid_until_us = valid_until_us;
    slot.occupied = true;
}

void Arbiter::expire(state::Micros now_us) noexcept
{
    for (std::size_t i = 0; i < state::kSourceCount; ++i) {
        Slot &slot = slots_[i];
        if (!slot.occupied || slot.valid_until_us == 0U) {
            continue;
        }
        if (now_us > slot.valid_until_us) {
            const SourceId source = static_cast<SourceId>(i);
            slot.occupied = false;
            slot.axis[0] = AxisRate{};
            slot.axis[1] = AxisRate{};
            if (owner_ == source) {
                owner_ = SourceId::None;
                ++generation_;
                owner_expired_ = true;
            }
            if (pending_claim_ == source) {
                cancelHandover();
            }
        }
    }
}

ArbitratedSetpoint Arbiter::arbitrate(state::Micros now_us) noexcept
{
    (void)now_us;
    ArbitratedSetpoint setpoint{};
    setpoint.owner = owner_;
    setpoint.owner_generation = generation_;
    if (handover_pending_) {
        setpoint.handover_pending = true;
        return setpoint;
    }
    if (owner_ == SourceId::None) {
        return setpoint;
    }
    const Slot &slot = slots_[indexOf(owner_)];
    if (!slot.occupied) {
        return setpoint;
    }
    setpoint.session = slot.session;
    for (std::size_t axis = 0; axis < 2; ++axis) {
        if (slot.axis[axis].valid) {
            setpoint.have_command = true;
            setpoint.rate[axis] = slot.axis[axis].half_steps_per_s;
        }
    }
    return setpoint;
}

bool Arbiter::revalidateClaim(state::Micros now_us, DenyReason &out_deny) const noexcept
{
    out_deny = DenyReason::None;
    if (!handover_pending_ || pending_claim_ == SourceId::None) {
        out_deny = DenyReason::NotOwner;
        return false;
    }
    const Slot &slot = slots_[indexOf(pending_claim_)];
    if (!slot.occupied) {
        out_deny = DenyReason::StaleInput;
        return false;
    }
    if (slot.valid_until_us != 0U && now_us > slot.valid_until_us) {
        out_deny = DenyReason::StaleInput;
        return false;
    }
    return true;
}

void Arbiter::completeHandover(state::Micros now_us) noexcept
{
    (void)now_us;
    if (!handover_pending_ || pending_claim_ == SourceId::None) {
        return;
    }
    owner_ = pending_claim_;
    pending_claim_ = SourceId::None;
    handover_pending_ = false;
    ++generation_;
}

void Arbiter::cancelHandover() noexcept
{
    pending_claim_ = SourceId::None;
    handover_pending_ = false;
}

void Arbiter::release(SourceId source, state::Micros now_us) noexcept
{
    (void)now_us;
    if (!validSource(source)) {
        return;
    }
    const std::size_t index = indexOf(source);
    slots_[index].occupied = false;
    slots_[index].axis[0] = AxisRate{};
    slots_[index].axis[1] = AxisRate{};
    if (owner_ == source) {
        owner_ = SourceId::None;
        ++generation_;
    }
    if (pending_claim_ == source) {
        cancelHandover();
    }
}

bool Arbiter::consumeOwnerExpired() noexcept
{
    const bool value = owner_expired_;
    owner_expired_ = false;
    return value;
}

}  // namespace control
