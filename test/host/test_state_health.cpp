/**
 * @file test_state_health.cpp
 * @brief Возраст, свежесть и здоровье наблюдаемых блоков.
 */

#include "check.hpp"
#include "state/health.hpp"

namespace {

using state::Micros;

void ageSaturates()
{
    CHECK(state::ageUs(10, 5) == 5U);
    CHECK(state::ageUs(5, 10) == 0U);  // сброс времени не даёт отрицания
}

void freshnessRules()
{
    CHECK(state::freshness(100, state::kNever, 10) == state::Freshness::Never);
    CHECK(state::freshness(100, 95, 10) == state::Freshness::Fresh);
    CHECK(state::freshness(100, 89, 10) == state::Freshness::Stale);
    CHECK(state::freshness(100, 101, 10) == state::Freshness::Stale);
}

void copyDoesNotRefresh()
{
    state::JoystickObservation obs{};
    obs.meta.valid = true;
    obs.meta.measured_us = 1000;
    obs.output_valid = true;
    obs.calibrated = true;

    // Повторное чтение старого снимка новым временем остаётся устаревшим.
    CHECK(state::freshness(2000, obs.meta.measured_us, 500) == state::Freshness::Stale);
    CHECK(state::joyHealth(obs, 2000, 500) == state::JoyHealth::Stale);
}

void joyHealthOrder()
{
    state::JoystickObservation obs{};
    obs.meta.valid = true;
    obs.meta.measured_us = 1000;
    obs.output_valid = false;
    obs.read_fault = true;
    // read_fault приоритетнее invalid.
    CHECK(state::joyHealth(obs, 1000, 500) == state::JoyHealth::Fault);

    obs.read_fault = false;
    CHECK(state::joyHealth(obs, 1000, 500) == state::JoyHealth::Invalid);

    obs.output_valid = true;
    CHECK(state::joyHealth(obs, 1000, 500) == state::JoyHealth::Ok);

    state::JoystickObservation empty{};
    CHECK(state::joyHealth(empty, 1000, 500) == state::JoyHealth::Unknown);
}

void axisHealthRules()
{
    state::AxisObservation obs{};
    CHECK(state::axisHealth(obs, 1000, 500) == state::AxisHealth::Unknown);

    obs.meta.valid = true;
    obs.meta.polled_us = 1000;
    obs.status.state = motion::AxisState::Idle;
    CHECK(state::axisHealth(obs, 1100, 500) == state::AxisHealth::Ok);

    obs.meta.polled_us = 100;
    CHECK(state::axisHealth(obs, 1100, 500) == state::AxisHealth::Stale);

    obs.meta.polled_us = 1100;
    obs.status.state = motion::AxisState::Disabled;
    CHECK(state::axisHealth(obs, 1100, 500) == state::AxisHealth::Disabled);

    obs.status.state = motion::AxisState::Fault;
    obs.status.has_fault = true;
    CHECK(state::axisHealth(obs, 1100, 500) == state::AxisHealth::Fault);
}

}  // namespace

int main()
{
    ageSaturates();
    freshnessRules();
    copyDoesNotRefresh();
    joyHealthOrder();
    axisHealthRules();
    return TEST_RESULT();
}
