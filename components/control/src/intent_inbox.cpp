/**
 * @file intent_inbox.cpp
 * @brief Атомарный приём намерения и разделение каналов.
 */

#include "control/intent_inbox.hpp"

namespace control {

bool IntentInbox::validSource(SourceId source) noexcept
{
    return source != SourceId::None && source != SourceId::Count;
}

std::size_t IntentInbox::indexOf(SourceId source) noexcept
{
    return static_cast<std::size_t>(source);
}

std::size_t IntentInbox::eventCount() const noexcept
{
    return count_;
}

bool IntentInbox::submit(const Intent &intent, Micros now_us,
                         DenyReason &out_deny) noexcept
{
    (void)now_us;
    out_deny = DenyReason::None;
    if (!validSource(intent.source)) {
        out_deny = DenyReason::InvalidConfig;
        return false;
    }
    const std::size_t index = indexOf(intent.source);
    if (intent.session == 0U) {
        out_deny = DenyReason::InvalidConfig;
        return false;
    }

    // 1. Одна проверка порядка сообщений на весь Intent.
    if (seen_[index]) {
        if (intent.session < last_session_[index]) {
            out_deny = DenyReason::OldSession;
            return false;
        }
        if (intent.session == last_session_[index] &&
            intent.sequence <= last_sequence_[index]) {
            out_deny = DenyReason::RepeatedSequence;
            return false;
        }
    }

    // 2. Проверка размещения обязательных событий владения.
    const bool wants_release = intent.release;
    const bool wants_claim = intent.claim && !wants_release;
    const std::size_t required_events =
        (wants_release ? 1U : 0U) + (wants_claim ? 1U : 0U);
    if (required_events > kEventCapacity - eventCount()) {
        out_deny = DenyReason::Busy;
        return false;  // целиком, без частичных изменений
    }

    // 3. Реестр обновляется РОВНО ОДИН раз на сообщение.
    seen_[index] = true;
    last_session_[index] = intent.session;
    last_sequence_[index] = intent.sequence;

    // 4. Разложение по каналам без повторного отбрасывания своих частей.
    rates_[index] = intent;
    rate_occupied_[index] = true;

    const std::size_t tail = (head_ + count_) % kEventCapacity;
    if (wants_release) {
        events_[tail] = Event{intent.source, EventKind::Release, intent.session,
                              intent.sequence, intent.created_us};
        ++count_;
    }
    if (wants_claim) {
        const std::size_t claim_tail = (head_ + count_) % kEventCapacity;
        events_[claim_tail] = Event{intent.source, EventKind::Claim, intent.session,
                                    intent.sequence, intent.created_us};
        ++count_;
    }

    if (intent.stop && !stop_latched_) {
        stop_latched_ = true;
        stop_ = StopRequest{intent.source, intent.session, intent.sequence,
                            intent.created_us};
    }
    return true;
}

std::size_t IntentInbox::drainRates(Intent *out, std::size_t max_count) noexcept
{
    std::size_t taken = 0;
    for (std::size_t i = 0; i < state::kSourceCount && taken < max_count; ++i) {
        if (rate_occupied_[i]) {
            out[taken++] = rates_[i];
            rate_occupied_[i] = false;
        }
    }
    return taken;
}

std::size_t IntentInbox::drainEvents(Event *out, std::size_t max_count) noexcept
{
    std::size_t taken = 0;
    while (taken < max_count && count_ > 0) {
        out[taken++] = events_[head_];
        head_ = (head_ + 1) % kEventCapacity;
        --count_;
    }
    return taken;
}

bool IntentInbox::takeStop(StopRequest &out) noexcept
{
    if (!stop_latched_) {
        return false;
    }
    out = stop_;
    stop_ = StopRequest{};
    stop_latched_ = false;
    return true;
}

void IntentInbox::clear() noexcept
{
    for (std::size_t i = 0; i < state::kSourceCount; ++i) {
        rates_[i] = Intent{};
        rate_occupied_[i] = false;
        last_session_[i] = 0;
        last_sequence_[i] = 0;
        seen_[i] = false;
    }
    head_ = 0;
    count_ = 0;
    stop_latched_ = false;
    stop_ = StopRequest{};
}

}  // namespace control
