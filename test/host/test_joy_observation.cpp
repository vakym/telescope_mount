/**
 * @file test_joy_observation.cpp
 * @brief Регрессия: снимок джойстика валиден для convert() и разрешает Enabling.
 *
 * Без явной установки meta.valid путь «исправный ADC -> нейтраль -> Enabling»
 * обрывался на NoSample, хотя хаб публиковал снимок как валидный.
 */

#include "check.hpp"
#include "control/arming.hpp"
#include "control/joy_mapping.hpp"
#include "control/joy_observation.hpp"

namespace {

using state::ControlPhase;
using state::Micros;

constexpr Micros kMeasured = 1'000'000;
constexpr Micros kPolled = 1'000'100;
constexpr Micros kStaleUs = 200'000;

control::JoystickSnapshotInput healthyNeutral()
{
    control::JoystickSnapshotInput input{};
    input.calibrated = true;
    input.sample_valid = true;
    input.output_valid = true;
    input.neutral = true;
    input.stick.neutral = true;
    input.measured_us = kMeasured;
    return input;
}

void observationIsValid()
{
    const state::JoystickObservation obs =
        control::makeObservation(healthyNeutral(), kPolled);
    CHECK(obs.meta.valid);
    CHECK(obs.meta.measured_us == kMeasured);
    CHECK(obs.meta.polled_us == kPolled);
}

void invalidObservationStaysInvalid()
{
    // Контрольный пример: снимок, созданный «пустым», невалиден по умолчанию.
    const state::JoystickObservation empty{};
    CHECK(!empty.meta.valid);

    control::JoyMappingConfig cfg{};
    cfg.stale_after_us = kStaleUs;
    CHECK(control::convert(empty, cfg, kPolled).deny == state::DenyReason::NoSample);
}

void neutralReachesEnabling()
{
    const state::JoystickObservation obs =
        control::makeObservation(healthyNeutral(), kPolled);

    control::JoyMappingConfig cfg{};
    cfg.stale_after_us = kStaleUs;
    const control::JoyConversion conversion = control::convert(obs, cfg, kPolled);
    CHECK(conversion.ok);
    CHECK(conversion.neutral);
    CHECK_NEAR(conversion.rate[0], 0.0, 1e-6);

    control::ArmingMachine machine;
    machine.configure({3});
    control::ArmingInputs inputs{};
    inputs.permit = true;
    (void)machine.step(inputs);  // Disarmed -> WaitingForNeutral
    inputs.neutral_confirmed =
        conversion.ok && conversion.neutral &&
        state::freshness(kPolled, obs.meta.measured_us, kStaleUs) ==
            state::Freshness::Fresh;
    CHECK(inputs.neutral_confirmed);

    const control::ArmingDecision decision = machine.step(inputs);
    CHECK(decision.phase == ControlPhase::Enabling);
    CHECK(decision.send_enable);
}

void staleObservationRejected()
{
    const state::JoystickObservation obs =
        control::makeObservation(healthyNeutral(), kPolled + kStaleUs + 1);
    control::JoyMappingConfig cfg{};
    cfg.stale_after_us = kStaleUs;
    CHECK(control::convert(obs, cfg, kPolled + kStaleUs + 1).deny ==
          state::DenyReason::StaleInput);
}

}  // namespace

int main()
{
    observationIsValid();
    invalidObservationStaysInvalid();
    neutralReachesEnabling();
    staleObservationRejected();
    return TEST_RESULT();
}
