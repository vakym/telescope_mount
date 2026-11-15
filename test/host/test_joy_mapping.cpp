/**
 * @file test_joy_mapping.cpp
 * @brief Преобразование положения стика в уставки: знак, масштаб, границы.
 */

#include <cmath>

#include "check.hpp"
#include "control/joy_mapping.hpp"

namespace {

using state::Micros;
constexpr Micros kNow = 1'000'000;
constexpr Micros kStaleUs = 200'000;

state::JoystickObservation makeSnapshot(float x, float y)
{
    state::JoystickObservation obs{};
    obs.meta.valid = true;
    obs.meta.measured_us = kNow;
    obs.calibrated = true;
    obs.sample_valid = true;
    obs.output_valid = true;
    obs.neutral = false;
    obs.stick.neutral = false;
    obs.stick.normalized[0] = x;
    obs.stick.normalized[1] = y;
    return obs;
}

control::JoyMappingConfig makeConfig()
{
    control::JoyMappingConfig cfg{};
    cfg.max_rate[0] = 800.0F;
    cfg.max_rate[1] = 800.0F;
    cfg.min_run_rate[0] = 40.0F;
    cfg.min_run_rate[1] = 40.0F;
    cfg.dead_zone = 0.12F;
    cfg.stale_after_us = kStaleUs;
    return cfg;
}

void fullDeflection()
{
    const control::JoyConversion out =
        control::convert(makeSnapshot(1.0F, 0.0F), makeConfig(), kNow);
    CHECK(out.ok);
    CHECK_NEAR(out.rate[0], 800.0, 1e-3);
    CHECK_NEAR(out.rate[1], 0.0, 1e-6);

    const control::JoyConversion negative =
        control::convert(makeSnapshot(0.0F, -1.0F), makeConfig(), kNow);
    CHECK(negative.ok);
    CHECK_NEAR(negative.rate[1], -800.0, 1e-3);
}

void deadZoneBoundary()
{
    const control::JoyMappingConfig cfg = makeConfig();
    // Ровно на границе компонентного гейта — ноль.
    const control::JoyConversion at_edge =
        control::convert(makeSnapshot(0.12F, 0.0F), cfg, kNow);
    CHECK(at_edge.ok);
    CHECK_NEAR(at_edge.rate[0], 0.0, 1e-6);

    // Чуть выше границы — скачок до минимальной скорости.
    const control::JoyConversion above =
        control::convert(makeSnapshot(0.125F, 0.0F), cfg, kNow);
    CHECK(above.ok);
    CHECK(above.rate[0] >= cfg.min_run_rate[0]);
    CHECK(above.rate[0] < cfg.min_run_rate[0] + 40.0F);
}

void diagonalAndComponentGate()
{
    const control::JoyMappingConfig cfg = makeConfig();
    const control::JoyConversion diagonal =
        control::convert(makeSnapshot(0.7F, -0.7F), cfg, kNow);
    CHECK(diagonal.ok);
    CHECK_GT(diagonal.rate[0], 0.0);
    CHECK_GT(-diagonal.rate[1], 0.0);

    // Компонент внутри мёртвой зоны не двигает ось даже при активном стике.
    const control::JoyConversion mostly_y =
        control::convert(makeSnapshot(0.085F, 0.6F), cfg, kNow);
    CHECK(mostly_y.ok);
    CHECK_NEAR(mostly_y.rate[0], 0.0, 1e-6);
    CHECK_GT(mostly_y.rate[1], 0.0);
}

void neutralGivesZero()
{
    state::JoystickObservation obs = makeSnapshot(0.9F, -0.4F);
    obs.neutral = true;
    obs.stick.neutral = true;
    const control::JoyConversion out = control::convert(obs, makeConfig(), kNow);
    CHECK(out.ok);
    CHECK(out.neutral);
    CHECK_NEAR(out.rate[0], 0.0, 1e-6);
    CHECK_NEAR(out.rate[1], 0.0, 1e-6);
}

void rejectionOrder()
{
    const control::JoyMappingConfig cfg = makeConfig();

    state::JoystickObservation no_sample = makeSnapshot(1.0F, 0.0F);
    no_sample.meta.valid = false;
    CHECK(control::convert(no_sample, cfg, kNow).deny == state::DenyReason::NoSample);

    state::JoystickObservation fault = makeSnapshot(1.0F, 0.0F);
    fault.read_fault = true;
    CHECK(control::convert(fault, cfg, kNow).deny == state::DenyReason::ReadFault);

    state::JoystickObservation invalid = makeSnapshot(1.0F, 0.0F);
    invalid.output_valid = false;
    CHECK(control::convert(invalid, cfg, kNow).deny == state::DenyReason::InvalidSample);

    state::JoystickObservation uncalibrated = makeSnapshot(1.0F, 0.0F);
    uncalibrated.calibrated = false;
    CHECK(control::convert(uncalibrated, cfg, kNow).deny ==
          state::DenyReason::NotCalibrated);

    // Устаревшее измерение при прочих валидных флагах.
    CHECK(control::convert(makeSnapshot(1.0F, 0.0F), cfg, kNow + kStaleUs + 1).deny ==
          state::DenyReason::StaleInput);

    state::JoystickObservation nan = makeSnapshot(std::nanf(""), 0.0F);
    CHECK(control::convert(nan, cfg, kNow).deny == state::DenyReason::InvalidSample);
}

void invalidConfiguration()
{
    control::JoyMappingConfig same_axis = makeConfig();
    same_axis.y_axis = same_axis.x_axis;
    CHECK(!control::isValid(same_axis));
    CHECK(control::convert(makeSnapshot(1.0F, 0.0F), same_axis, kNow).deny ==
          state::DenyReason::InvalidConfig);

    control::JoyMappingConfig bad_min = makeConfig();
    bad_min.min_run_rate[0] = bad_min.max_rate[0];
    CHECK(!control::isValid(bad_min));

    control::JoyMappingConfig bad_dz = makeConfig();
    bad_dz.dead_zone = 1.0F;
    CHECK(!control::isValid(bad_dz));

    control::JoyMappingConfig count_axis = makeConfig();
    count_axis.x_axis = motion::AxisId::Count;
    CHECK(!control::isValid(count_axis));
}

void inversion()
{
    control::JoyMappingConfig cfg = makeConfig();
    cfg.invert_x = true;
    const control::JoyConversion out =
        control::convert(makeSnapshot(1.0F, 0.0F), cfg, kNow);
    CHECK(out.ok);
    CHECK(out.rate[0] < 0.0F);
}

}  // namespace

int main()
{
    fullDeflection();
    deadZoneBoundary();
    diagonalAndComponentGate();
    neutralGivesZero();
    rejectionOrder();
    invalidConfiguration();
    inversion();
    return TEST_RESULT();
}
