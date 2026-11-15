/**
 * @file test_arbiter.cpp
 * @brief Владение парой осей: приоритеты, передача, lease, отказы.
 */

#include "check.hpp"
#include "control/arbiter.hpp"

namespace {

using control::Arbiter;
using control::AxisRate;
using state::DenyReason;
using state::SourceId;

AxisRate (&rates100(AxisRate (&buffer)[2]))[2]
{
    buffer[0].valid = true;
    buffer[0].half_steps_per_s = 100.0F;
    buffer[1].valid = false;
    buffer[1].half_steps_per_s = 0.0F;
    return buffer;
}

void configure(Arbiter &arbiter)
{
    arbiter.configure({1000, 3000});
}

void claimGrantsOwnership()
{
    Arbiter arbiter;
    configure(arbiter);
    AxisRate rates[2]{};
    DenyReason deny = DenyReason::None;
    arbiter.applyRates(SourceId::Joystick, rates100(rates), 1, 0, 5000);
    arbiter.applyOwnership(SourceId::Joystick, true, false, 1, 0, deny);
    CHECK(deny == DenyReason::None);
    CHECK(arbiter.owner() == SourceId::Joystick);
    const std::uint32_t generation = arbiter.generation();

    const control::ArbitratedSetpoint setpoint = arbiter.arbitrate(0);
    CHECK(setpoint.have_command);
    CHECK_NEAR(setpoint.rate[0], 100.0, 1e-3);
    CHECK(setpoint.owner_generation == generation);
}

void higherPriorityPreempts()
{
    Arbiter arbiter;
    configure(arbiter);
    AxisRate rates[2]{};
    DenyReason deny = DenyReason::None;
    arbiter.applyRates(SourceId::Browser, rates100(rates), 1, 0, 5000);
    arbiter.applyOwnership(SourceId::Browser, true, false, 1, 0, deny);

    arbiter.applyRates(SourceId::Joystick, rates100(rates), 1, 0, 5000);
    arbiter.applyOwnership(SourceId::Joystick, true, false, 1, 0, deny);
    CHECK(arbiter.handoverPending());
    CHECK(arbiter.owner() == SourceId::None);
    CHECK(arbiter.arbitrate(0).handover_pending);
    CHECK(!arbiter.arbitrate(0).have_command);

    // Пока передача не завершена, другой источник не захватывает владение.
    arbiter.applyOwnership(SourceId::Automation, true, false, 1, 0, deny);
    CHECK(deny == DenyReason::StopInProgress);

    CHECK(arbiter.revalidateClaim(0, deny));
    arbiter.completeHandover(0);
    CHECK(arbiter.owner() == SourceId::Joystick);
    CHECK(!arbiter.handoverPending());
}

void lowerPriorityRejected()
{
    Arbiter arbiter;
    configure(arbiter);
    AxisRate rates[2]{};
    DenyReason deny = DenyReason::None;
    arbiter.applyRates(SourceId::Browser, rates100(rates), 1, 0, 5000);
    arbiter.applyOwnership(SourceId::Browser, true, false, 1, 0, deny);

    arbiter.applyOwnership(SourceId::Automation, true, false, 1, 0, deny);
    CHECK(deny == DenyReason::NotOwner);
    CHECK(arbiter.owner() == SourceId::Browser);
}

void releaseGivesUpOwnership()
{
    Arbiter arbiter;
    configure(arbiter);
    AxisRate rates[2]{};
    DenyReason deny = DenyReason::None;
    arbiter.applyRates(SourceId::Joystick, rates100(rates), 1, 0, 5000);
    arbiter.applyOwnership(SourceId::Joystick, true, false, 1, 0, deny);
    arbiter.applyOwnership(SourceId::Joystick, false, true, 1, 0, deny);
    CHECK(arbiter.owner() == SourceId::None);
    CHECK(!arbiter.arbitrate(0).have_command);
    // Пассивная нейтраль не возобновляет претензию: нужен новый claim.
    arbiter.applyOwnership(SourceId::Joystick, false, false, 1, 0, deny);
    CHECK(arbiter.owner() == SourceId::None);
    CHECK(deny == DenyReason::NotOwner);
}

void leaseExpiry()
{
    Arbiter arbiter;
    configure(arbiter);
    AxisRate rates[2]{};
    DenyReason deny = DenyReason::None;
    arbiter.applyRates(SourceId::Joystick, rates100(rates), 1, 0, 100);
    arbiter.applyOwnership(SourceId::Joystick, true, false, 1, 0, deny);
    CHECK(arbiter.owner() == SourceId::Joystick);

    arbiter.expire(200);
    CHECK(arbiter.owner() == SourceId::None);
    CHECK(arbiter.consumeOwnerExpired());
    CHECK(!arbiter.consumeOwnerExpired());
}

void staleClaimRevalidation()
{
    Arbiter arbiter;
    configure(arbiter);
    AxisRate rates[2]{};
    DenyReason deny = DenyReason::None;
    arbiter.applyRates(SourceId::Browser, rates100(rates), 1, 0, 5000);
    arbiter.applyOwnership(SourceId::Browser, true, false, 1, 0, deny);

    arbiter.applyRates(SourceId::Joystick, rates100(rates), 1, 0, 50);
    arbiter.applyOwnership(SourceId::Joystick, true, false, 1, 0, deny);
    CHECK(arbiter.handoverPending());

    // Претендент истёк за время торможения — передача не выполняется.
    CHECK(!arbiter.revalidateClaim(100, deny));
    CHECK(deny == DenyReason::StaleInput);
    arbiter.cancelHandover();
    CHECK(arbiter.owner() == SourceId::None);
}

void olderSessionRatesIgnored()
{
    Arbiter arbiter;
    configure(arbiter);
    AxisRate newer[2]{};
    newer[0].valid = true;
    newer[0].half_steps_per_s = 300.0F;
    AxisRate older[2]{};
    older[0].valid = true;
    older[0].half_steps_per_s = 50.0F;

    DenyReason deny = DenyReason::None;
    arbiter.applyOwnership(SourceId::Joystick, true, false, 2, 0, deny);
    arbiter.applyRates(SourceId::Joystick, newer, 2, 0, 5000);
    arbiter.applyRates(SourceId::Joystick, older, 1, 0, 5000);
    CHECK_NEAR(arbiter.arbitrate(0).rate[0], 300.0, 1e-3);
}

}  // namespace

int main()
{
    claimGrantsOwnership();
    higherPriorityPreempts();
    lowerPriorityRejected();
    releaseGivesUpOwnership();
    leaseExpiry();
    staleClaimRevalidation();
    olderSessionRatesIgnored();
    return TEST_RESULT();
}
