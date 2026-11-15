/**
 * @file test_intent_inbox.cpp
 * @brief Атомарный приём намерения и разделение каналов.
 */

#include "check.hpp"
#include "control/intent_inbox.hpp"

namespace {

using control::Intent;
using control::IntentInbox;
using state::DenyReason;
using state::SourceId;

Intent makeIntent(SourceId source, std::uint32_t session, std::uint32_t sequence)
{
    Intent intent{};
    intent.source = source;
    intent.session = session;
    intent.sequence = sequence;
    intent.created_us = 1000;
    intent.valid_until_us = 2000;
    intent.axis[0].valid = true;
    intent.axis[0].half_steps_per_s = 100.0F;
    return intent;
}

void atomicAccept()
{
    IntentInbox inbox;
    Intent intent = makeIntent(SourceId::Browser, 1, 1);
    intent.claim = true;
    intent.stop = true;
    DenyReason deny = DenyReason::None;
    CHECK(inbox.submit(intent, 0, deny));
    CHECK(deny == DenyReason::None);

    IntentInbox::StopRequest stop{};
    CHECK(inbox.takeStop(stop));
    CHECK(stop.source == SourceId::Browser);
    CHECK(stop.sequence == 1);
    CHECK(!inbox.stopLatched());

    IntentInbox::Event events[IntentInbox::kEventCapacity]{};
    CHECK(inbox.drainEvents(events, IntentInbox::kEventCapacity) == 1U);
    CHECK(events[0].kind == IntentInbox::EventKind::Claim);
    CHECK(events[0].sequence == 1);

    Intent rates[state::kSourceCount]{};
    CHECK(inbox.drainRates(rates, state::kSourceCount) == 1U);
    CHECK_NEAR(rates[0].axis[0].half_steps_per_s, 100.0, 1e-3);
}

void repeatedSequenceRejected()
{
    IntentInbox inbox;
    DenyReason deny = DenyReason::None;
    CHECK(inbox.submit(makeIntent(SourceId::Joystick, 1, 2), 0, deny));
    CHECK(!inbox.submit(makeIntent(SourceId::Joystick, 1, 2), 0, deny));
    CHECK(deny == DenyReason::RepeatedSequence);
    CHECK(!inbox.submit(makeIntent(SourceId::Joystick, 1, 1), 0, deny));
    CHECK(deny == DenyReason::RepeatedSequence);
    CHECK(inbox.submit(makeIntent(SourceId::Joystick, 1, 3), 0, deny));
}

void oldSessionRejected()
{
    IntentInbox inbox;
    DenyReason deny = DenyReason::None;
    CHECK(inbox.submit(makeIntent(SourceId::Browser, 5, 1), 0, deny));
    CHECK(!inbox.submit(makeIntent(SourceId::Browser, 4, 9), 0, deny));
    CHECK(deny == DenyReason::OldSession);
}

void claimSurvivesLaterRate()
{
    IntentInbox inbox;
    DenyReason deny = DenyReason::None;
    Intent claim = makeIntent(SourceId::Browser, 1, 10);
    claim.claim = true;
    CHECK(inbox.submit(claim, 0, deny));
    // Более новое сообщение без claim не должно съесть уже принятое событие.
    CHECK(inbox.submit(makeIntent(SourceId::Browser, 1, 11), 0, deny));

    IntentInbox::Event events[IntentInbox::kEventCapacity]{};
    CHECK(inbox.drainEvents(events, IntentInbox::kEventCapacity) == 1U);
    CHECK(events[0].sequence == 10);
}

void stopNotOverwrittenByRate()
{
    IntentInbox inbox;
    DenyReason deny = DenyReason::None;
    Intent stopper = makeIntent(SourceId::Joystick, 1, 1);
    stopper.stop = true;
    CHECK(inbox.submit(stopper, 0, deny));
    CHECK(inbox.submit(makeIntent(SourceId::Joystick, 1, 2), 0, deny));

    IntentInbox::StopRequest stop{};
    CHECK(inbox.takeStop(stop));
    CHECK(stop.sequence == 1);
}

void ringFullRejectedAtomically()
{
    IntentInbox inbox;
    DenyReason deny = DenyReason::None;
    for (std::uint32_t seq = 1; seq <= IntentInbox::kEventCapacity; ++seq) {
        Intent claim = makeIntent(SourceId::Automation, 1, seq);
        claim.claim = true;
        CHECK(inbox.submit(claim, 0, deny));
    }
    // Места под событие нет: сообщение отклоняется целиком.
    Intent overflow = makeIntent(SourceId::Automation, 1, 5);
    overflow.claim = true;
    overflow.axis[0].half_steps_per_s = 777.0F;
    CHECK(!inbox.submit(overflow, 0, deny));
    CHECK(deny == DenyReason::Busy);

    // Реестр не изменился: сообщение без события принимается с тем же seq.
    Intent rates = makeIntent(SourceId::Automation, 1, 5);
    rates.axis[0].half_steps_per_s = 555.0F;
    CHECK(inbox.submit(rates, 0, deny));

    Intent drained[state::kSourceCount]{};
    CHECK(inbox.drainRates(drained, state::kSourceCount) == 1U);
    CHECK_NEAR(drained[0].axis[0].half_steps_per_s, 555.0, 1e-3);
}

void invalidSourceRejected()
{
    IntentInbox inbox;
    DenyReason deny = DenyReason::None;
    Intent intent = makeIntent(SourceId::None, 1, 1);
    CHECK(!inbox.submit(intent, 0, deny));
    CHECK(deny == DenyReason::InvalidConfig);
}

}  // namespace

int main()
{
    atomicAccept();
    repeatedSequenceRejected();
    oldSessionRejected();
    claimSurvivesLaterRate();
    stopNotOverwrittenByRate();
    ringFullRejectedAtomically();
    invalidSourceRejected();
    return TEST_RESULT();
}
