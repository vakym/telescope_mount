/**
 * @file test_input_grace.cpp
 * @brief Grace только для единичной ошибки ADC и только в пределах срока.
 */

#include "check.hpp"
#include "control/input_grace.hpp"

namespace {

using state::DenyReason;
using state::Micros;

constexpr Micros kGrace = 100'000;
constexpr Micros kStale = 500'000;

bool configure(control::InputGracePolicy &policy)
{
    return policy.configure({kGrace}, kStale);
}

void deadlineNotExtended()
{
    control::InputGracePolicy policy;
    CHECK(configure(policy));
    policy.onGoodSample(1'000'000);

    const auto first = policy.onRejected(DenyReason::InvalidSample, 1'020'000);
    CHECK(first.hold_previous);
    const auto second = policy.onRejected(DenyReason::InvalidSample, 1'090'000);
    CHECK(second.hold_previous);  // срок не продлевается
    const auto expired = policy.onRejected(DenyReason::InvalidSample, 1'120'000);
    CHECK(!expired.hold_previous);
    CHECK(expired.expired);
}

void goodSampleResets()
{
    control::InputGracePolicy policy;
    CHECK(configure(policy));
    policy.onGoodSample(1'000'000);
    (void)policy.onRejected(DenyReason::InvalidSample, 1'050'000);
    policy.onGoodSample(1'060'000);
    const auto again = policy.onRejected(DenyReason::InvalidSample, 1'070'000);
    CHECK(again.hold_previous);
}

void onlyInvalidSampleIsAllowed()
{
    control::InputGracePolicy policy;
    CHECK(configure(policy));
    policy.onGoodSample(1'000'000);

    CHECK(policy.onRejected(DenyReason::ReadFault, 1'010'000).expired);
    CHECK(policy.onRejected(DenyReason::StaleInput, 1'010'000).expired);
    CHECK(policy.onRejected(DenyReason::InvalidConfig, 1'010'000).expired);
    CHECK(policy.onRejected(DenyReason::NotCalibrated, 1'010'000).expired);
    CHECK(policy.onRejected(DenyReason::NotOwner, 1'010'000).expired);
}

void noGoodSampleMeansNoGrace()
{
    control::InputGracePolicy policy;
    CHECK(configure(policy));
    const auto decision = policy.onRejected(DenyReason::InvalidSample, 1'000'000);
    CHECK(!decision.hold_previous);
    CHECK(decision.expired);
}

void limitedByMeasurementAge()
{
    control::InputGracePolicy policy;
    CHECK(policy.configure({100'000}, 150'000));
    policy.onGoodSample(1'000'000);  // допустимый возраст до 1'150'000
    (void)policy.onRejected(DenyReason::InvalidSample, 1'080'000);
    CHECK(policy.deadline() == 1'150'000);

    const auto at_limit = policy.onRejected(DenyReason::InvalidSample, 1'150'000);
    CHECK(!at_limit.hold_previous);
    CHECK(at_limit.expired);
}

void invalidConfigRejected()
{
    control::InputGracePolicy policy;
    CHECK(!policy.configure({0}, kStale));
    CHECK(!policy.configure({kGrace}, 0));
}

}  // namespace

int main()
{
    deadlineNotExtended();
    goodSampleResets();
    onlyInvalidSampleIsAllowed();
    noGoodSampleMeansNoGrace();
    limitedByMeasurementAge();
    invalidConfigRejected();
    return TEST_RESULT();
}
