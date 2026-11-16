/**
 * @file test_state_json.cpp
 * @brief Host-тесты контракта и ограниченной сериализации состояния.
 *
 * Проверяются: имена enum, единицы, политика null, NaN/Inf, границы
 * диапазона скорости, максимальные значения, ёмкость буфера и худший случай.
 */

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

#include "check.hpp"
#include "web/state_json.hpp"

namespace {

using state::AxisObservation;
using state::ControlObservation;
using state::JoystickObservation;
using state::Micros;
using state::SystemSnapshot;

constexpr Micros kNow = 10'000'000;  // 10 с после старта

bool has(const char *doc, const char *needle)
{
    return std::strstr(doc, needle) != nullptr;
}

/// Проверка внутри блока одной оси: поиск ограничен до закрывающей скобки.
bool axisHas(const char *doc, const char *axis_name, const char *needle)
{
    const char *start = std::strstr(doc, axis_name);
    if (start == nullptr) {
        return false;
    }
    const char *end = std::strchr(start, '}');
    if (end == nullptr) {
        return false;
    }
    const char *found = std::strstr(start, needle);
    return found != nullptr && found < end;
}

/// Грубая проверка структуры: без пустых элементов и висячих запятых.
bool structurally_sane(const char *doc)
{
    const std::size_t size = std::strlen(doc);
    if (size < 2 || doc[0] != '{' || doc[size - 1] != '}') {
        return false;
    }
    return !has(doc, ",}") && !has(doc, ",,") && !has(doc, "}{");
}

SystemSnapshot makePopulated()
{
    SystemSnapshot snapshot{};
    snapshot.generated_us = kNow;

    AxisObservation &azimuth = snapshot.axis[0];
    azimuth.meta.valid = true;
    azimuth.meta.polled_us = kNow - 12'000;
    azimuth.status.state = motion::AxisState::Idle;
    azimuth.status.position_step = 4096;
    azimuth.status.rate_steps_s = 0.0F;
    azimuth.status.energized = false;
    azimuth.status.position_trusted = true;

    AxisObservation &altitude = snapshot.axis[1];
    altitude.meta.valid = true;
    altitude.meta.polled_us = kNow - 40'000;
    altitude.status.state = motion::AxisState::Cruising;
    altitude.status.position_step = -1234;
    altitude.status.rate_steps_s = -320.5F;
    altitude.status.energized = true;
    altitude.status.position_trusted = true;
    altitude.status.late_corrections = 2;
    altitude.status.rejected_commands = 7;

    snapshot.joystick.meta.valid = true;
    snapshot.joystick.meta.measured_us = kNow - 8'000;
    snapshot.joystick.calibrated = true;
    snapshot.joystick.output_valid = true;
    snapshot.joystick.neutral = true;
    snapshot.joystick.read_errors = 1;

    snapshot.control.meta.valid = true;
    snapshot.control.meta.polled_us = kNow - 20'000;
    snapshot.control.phase = state::ControlPhase::WaitingForNeutral;
    snapshot.control.latch = state::LatchCode::None;
    snapshot.control.deny = state::DenyReason::StaleInput;
    snapshot.control.permit = false;
    snapshot.control.owner = state::SourceId::Joystick;
    snapshot.control.owner_generation = 3;
    snapshot.control.delivery_failures[0] = 1;
    snapshot.control.delivery_failures[1] = 2;
    return snapshot;
}

web::SerializeOptions makeOptions()
{
    web::SerializeOptions opts;
    opts.axis_stale_us = 500'000;
    opts.joystick_stale_us = 500'000;
    opts.control_stale_us = 500'000;
    return opts;
}

struct Out {
    char buffer[web::kMaxStateJsonBytes]{};
    std::size_t written = 0;
    web::SerializeResult result = web::SerializeResult::Overflow;

    explicit Out(std::size_t capacity = sizeof(buffer))
        : result(web::stateJson(makePopulated(), makeOptions(), buffer, capacity,
                                &written))
    {
    }

    Out(const SystemSnapshot &snapshot, std::size_t capacity)
        : result(web::stateJson(snapshot, makeOptions(), buffer, capacity,
                                &written))
    {
    }

    Out(const SystemSnapshot &snapshot, const web::SerializeOptions &options,
        std::size_t capacity)
        : result(web::stateJson(snapshot, options, buffer, capacity, &written))
    {
    }

    const char *text() const { return buffer; }
};

void testPopulated()
{
    const Out out;
    CHECK(out.result == web::SerializeResult::Ok);
    CHECK(out.written == std::strlen(out.text()));
    CHECK(structurally_sane(out.text()));
    CHECK(has(out.text(), "\"schema\":1"));
    CHECK(has(out.text(), "\"generated_ms\":10000"));
    CHECK(has(out.text(), "\"phase\":\"waiting_neutral\""));
    CHECK(has(out.text(), "\"latch\":\"none\""));
    CHECK(has(out.text(), "\"deny\":\"stale_input\""));
    CHECK(has(out.text(), "\"owner\":\"joystick\""));
    CHECK(has(out.text(), "\"generation\":3"));
    CHECK(axisHas(out.text(), "\"azimuth\"", "\"health\":\"ok\""));
    CHECK(axisHas(out.text(), "\"azimuth\"", "\"state\":\"idle\""));
    CHECK(axisHas(out.text(), "\"azimuth\"", "\"position_half_steps\":\"4096\""));
    CHECK(axisHas(out.text(), "\"azimuth\"", "\"age_ms\":12"));
    CHECK(axisHas(out.text(), "\"azimuth\"", "\"rate_half_steps_s\":0.0"));
    CHECK(axisHas(out.text(), "\"altitude\"", "\"state\":\"cruising\""));
    CHECK(axisHas(out.text(), "\"altitude\"", "\"position_half_steps\":\"-1234\""));
    CHECK(axisHas(out.text(), "\"altitude\"", "\"rate_half_steps_s\":-320.5"));
    CHECK(axisHas(out.text(), "\"altitude\"", "\"late_corrections\":\"2\""));
    CHECK(axisHas(out.text(), "\"altitude\"", "\"rejected_commands\":\"7\""));
    CHECK(has(out.text(), "\"neutral\":true"));
    CHECK(has(out.text(), "\"delivery_failures\":[1,2]"));
    CHECK(has(out.text(), "\"fault_reason\":null"));
    CHECK(!has(out.text(), "deg"));
}

void testEmptySnapshot()
{
    const SystemSnapshot snapshot{};
    const Out out(snapshot, sizeof(out.buffer));
    CHECK(out.result == web::SerializeResult::Ok);
    CHECK(structurally_sane(out.text()));
    CHECK(has(out.text(), "\"control\":{\"valid\":false,\"health\":\"unknown\",\"age_ms\":null"));
    CHECK(has(out.text(), "\"phase\":null"));
    CHECK(has(out.text(), "\"permit\":null"));
    CHECK(has(out.text(), "\"position_half_steps\":null"));
    CHECK(has(out.text(), "\"rate_half_steps_s\":null"));
    CHECK(has(out.text(), "\"stop_timeouts\":null"));
    CHECK(!has(out.text(), "\"phase\":\"disarmed\""));
    CHECK(!has(out.text(), "\"owner\":\"none\""));
}

void testNonFiniteRates()
{
    SystemSnapshot nan_snapshot = makePopulated();
    nan_snapshot.axis[0].status.rate_steps_s =
        std::numeric_limits<float>::quiet_NaN();
    Out nan_out(nan_snapshot, sizeof(nan_out.buffer));
    CHECK(nan_out.result == web::SerializeResult::Ok);
    CHECK(axisHas(nan_out.text(), "\"azimuth\"", "\"rate_half_steps_s\":null"));
    CHECK(axisHas(nan_out.text(), "\"azimuth\"", "\"position_half_steps\":\"4096\""));

    SystemSnapshot inf_snapshot = makePopulated();
    inf_snapshot.axis[0].status.rate_steps_s =
        std::numeric_limits<float>::infinity();
    Out inf_out(inf_snapshot, sizeof(inf_out.buffer));
    CHECK(axisHas(inf_out.text(), "\"azimuth\"", "\"rate_half_steps_s\":null"));

    SystemSnapshot neg_inf_snapshot = makePopulated();
    neg_inf_snapshot.axis[0].status.rate_steps_s =
        -std::numeric_limits<float>::infinity();
    Out neg_out(neg_inf_snapshot, sizeof(neg_out.buffer));
    CHECK(axisHas(neg_out.text(), "\"azimuth\"", "\"rate_half_steps_s\":null"));
    CHECK(structurally_sane(neg_out.text()));
}

void testRateFormatting()
{
    struct Case {
        float rate;
        const char *expected;
    };
    const Case cases[] = {
        {-0.0F, "\"rate_half_steps_s\":0.0"},
        {0.0F, "\"rate_half_steps_s\":0.0"},
        {-0.1F, "\"rate_half_steps_s\":-0.1"},
        {0.05F, "\"rate_half_steps_s\":0.1"},
        {0.049999F, "\"rate_half_steps_s\":0.0"},
        {-0.05F, "\"rate_half_steps_s\":-0.1"},
        {10000.0F, "\"rate_half_steps_s\":10000.0"},
        {-10000.0F, "\"rate_half_steps_s\":-10000.0"},
    };
    for (const Case &item : cases) {
        SystemSnapshot snapshot = makePopulated();
        snapshot.axis[0].status.rate_steps_s = item.rate;
        Out out(snapshot, sizeof(out.buffer));
        CHECK(out.result == web::SerializeResult::Ok);
        if (!axisHas(out.text(), "\"azimuth\"", item.expected)) {
            std::fprintf(stderr, "rate %.6f -> %s\n",
                         static_cast<double>(item.rate), out.text());
        }
        CHECK(axisHas(out.text(), "\"azimuth\"", item.expected));
    }

    const float rejected[] = {
        10000.1F,
        -10000.1F,
        1.0e30F,
        std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
    };
    for (float rate : rejected) {
        SystemSnapshot snapshot = makePopulated();
        snapshot.axis[0].status.rate_steps_s = rate;
        Out out(snapshot, sizeof(out.buffer));
        CHECK(out.result == web::SerializeResult::Ok);
        CHECK(axisHas(out.text(), "\"azimuth\"", "\"rate_half_steps_s\":null"));
        CHECK(structurally_sane(out.text()));
    }

    SystemSnapshot tiny = makePopulated();
    tiny.axis[0].status.rate_steps_s = std::numeric_limits<float>::min();
    Out tiny_out(tiny, sizeof(tiny_out.buffer));
    CHECK(axisHas(tiny_out.text(), "\"azimuth\"", "\"rate_half_steps_s\":0.0"));
}

void testMaxValues()
{
    SystemSnapshot snapshot = makePopulated();
    snapshot.generated_us = 9'000'000'000'000ULL;
    snapshot.axis[0].status.position_step =
        std::numeric_limits<std::int64_t>::min();
    snapshot.axis[1].status.position_step =
        std::numeric_limits<std::int64_t>::max();
    snapshot.axis[0].status.late_corrections =
        std::numeric_limits<std::uint64_t>::max();
    snapshot.axis[1].status.late_replans =
        std::numeric_limits<std::uint64_t>::max();
    snapshot.control.owner_generation =
        std::numeric_limits<std::uint32_t>::max();
    snapshot.control.stop_timeouts = std::numeric_limits<std::uint32_t>::max();
    snapshot.axis[0].status.has_fault = true;
    snapshot.axis[0].status.fault_reason = motion::FaultReason::WatchdogTimeout;

    const Out out(snapshot, sizeof(out.buffer));
    CHECK(out.result == web::SerializeResult::Ok);
    CHECK(structurally_sane(out.text()));
    CHECK(axisHas(out.text(), "\"azimuth\"",
                  "\"position_half_steps\":\"-9223372036854775808\""));
    CHECK(axisHas(out.text(), "\"altitude\"",
                  "\"position_half_steps\":\"9223372036854775807\""));
    CHECK(axisHas(out.text(), "\"azimuth\"",
                  "\"late_corrections\":\"18446744073709551615\""));
    CHECK(axisHas(out.text(), "\"altitude\"",
                  "\"late_replans\":\"18446744073709551615\""));
    CHECK(has(out.text(), "\"generation\":4294967295"));
    CHECK(has(out.text(), "\"stop_timeouts\":4294967295"));
    CHECK(axisHas(out.text(), "\"azimuth\"", "\"fault\":true"));
    CHECK(axisHas(out.text(), "\"azimuth\"",
                  "\"fault_reason\":\"watchdog_timeout\""));
}

void testAges()
{
    SystemSnapshot snapshot = makePopulated();
    snapshot.axis[0].meta.polled_us = state::kNever;
    Out never_out(snapshot, sizeof(never_out.buffer));
    CHECK(axisHas(never_out.text(), "\"azimuth\"", "\"age_ms\":null"));
    CHECK(axisHas(never_out.text(), "\"azimuth\"", "\"health\":\"stale\""));

    SystemSnapshot future = makePopulated();
    future.axis[0].meta.polled_us = kNow + 1000;
    Out future_out(future, sizeof(future_out.buffer));
    CHECK(axisHas(future_out.text(), "\"azimuth\"", "\"age_ms\":null"));
    CHECK(!has(future_out.text(), "\"age_ms\":-"));

    SystemSnapshot rounded = makePopulated();
    rounded.axis[0].meta.polled_us = kNow - 1'500'000;
    Out rounded_out(rounded, sizeof(rounded_out.buffer));
    CHECK(axisHas(rounded_out.text(), "\"azimuth\"", "\"age_ms\":1500"));
}

void testFreshnessBoundary()
{
    SystemSnapshot at_limit = makePopulated();
    at_limit.axis[0].meta.polled_us = kNow - 500'000;  // ровно порог
    Out limit_out(at_limit, sizeof(limit_out.buffer));
    CHECK(axisHas(limit_out.text(), "\"azimuth\"", "\"health\":\"ok\""));

    SystemSnapshot over_limit = makePopulated();
    over_limit.axis[0].meta.polled_us = kNow - 500'001;
    Out over_out(over_limit, sizeof(over_out.buffer));
    CHECK(axisHas(over_out.text(), "\"azimuth\"", "\"health\":\"stale\""));
    // Устаревший блок сохраняет значения.
    CHECK(axisHas(over_out.text(), "\"azimuth\"",
                  "\"position_half_steps\":\"4096\""));
    CHECK(axisHas(over_out.text(), "\"azimuth\"", "\"valid\":true"));
}

void testInvalidatedBlocks()
{
    SystemSnapshot snapshot = makePopulated();
    snapshot.axis[0].meta.valid = false;
    snapshot.joystick.meta.valid = false;
    snapshot.control.meta.valid = false;
    const Out out(snapshot, sizeof(out.buffer));
    CHECK(out.result == web::SerializeResult::Ok);
    CHECK(structurally_sane(out.text()));
    CHECK(axisHas(out.text(), "\"azimuth\"", "\"position_half_steps\":null"));
    CHECK(!axisHas(out.text(), "\"azimuth\"", "\"position_half_steps\":\"4096\""));
    CHECK(has(out.text(), "\"neutral\":null"));
    CHECK(has(out.text(), "\"phase\":null"));
    CHECK(has(out.text(), "\"permit\":null"));
    CHECK(has(out.text(), "\"delivery_failures\":null"));
    // Второй (валидный) блок остаётся на месте.
    CHECK(axisHas(out.text(), "\"altitude\"",
                  "\"position_half_steps\":\"-1234\""));
}

void testUnknownEnums()
{
    SystemSnapshot snapshot = makePopulated();
    snapshot.control.phase = static_cast<state::ControlPhase>(200);
    snapshot.control.latch = static_cast<state::LatchCode>(200);
    snapshot.control.deny = static_cast<state::DenyReason>(200);
    snapshot.control.owner = static_cast<state::SourceId>(200);
    snapshot.axis[0].status.state = static_cast<motion::AxisState>(200);
    const Out out(snapshot, sizeof(out.buffer));
    CHECK(out.result == web::SerializeResult::Ok);
    CHECK(structurally_sane(out.text()));
    CHECK(has(out.text(), "\"phase\":\"unknown\""));
    CHECK(has(out.text(), "\"latch\":\"unknown\""));
    CHECK(has(out.text(), "\"deny\":\"unknown\""));
    CHECK(has(out.text(), "\"owner\":\"unknown\""));
    CHECK(axisHas(out.text(), "\"azimuth\"", "\"state\":\"unknown\""));
}

void testControlAgeAndHealth()
{
    SystemSnapshot fresh = makePopulated();
    Out fresh_out(fresh, sizeof(fresh_out.buffer));
    CHECK(has(fresh_out.text(),
              "\"control\":{\"valid\":true,\"health\":\"ok\",\"age_ms\":20"));

    SystemSnapshot stale = makePopulated();
    stale.control.meta.polled_us = kNow - 2'000'000;
    Out stale_out(stale, sizeof(stale_out.buffer));
    CHECK(has(stale_out.text(),
              "\"control\":{\"valid\":true,\"health\":\"stale\",\"age_ms\":2000"));
    // Устаревшее управление не обнуляет фазу.
    CHECK(has(stale_out.text(), "\"phase\":\"waiting_neutral\""));

    SystemSnapshot no_poll = makePopulated();
    no_poll.control.meta.polled_us = state::kNever;
    Out no_poll_out(no_poll, sizeof(no_poll_out.buffer));
    CHECK(has(no_poll_out.text(), "\"health\":\"unknown\""));
    CHECK(has(no_poll_out.text(), "\"phase\":\"waiting_neutral\""));
}

void testCapacity()
{
    Out full(makePopulated(), sizeof(Out::buffer));
    CHECK(full.result == web::SerializeResult::Ok);
    const std::size_t needed = full.written + 1;  // включая завершающий нуль

    Out exact(makePopulated(), needed);
    CHECK(exact.result == web::SerializeResult::Ok);
    CHECK(exact.written == full.written);

    Out too_small(makePopulated(), needed - 1);
    CHECK(too_small.result == web::SerializeResult::Overflow);
    CHECK(too_small.written == 0);
    CHECK(too_small.text()[0] == '\0');

    Out zero(makePopulated(), 0);
    CHECK(zero.result == web::SerializeResult::Overflow);
}

void testWorstCaseFits()
{
    SystemSnapshot snapshot = makePopulated();
    snapshot.generated_us = std::numeric_limits<std::uint64_t>::max();
    snapshot.control.phase = state::ControlPhase::WaitingForNeutral;
    snapshot.control.latch = state::LatchCode::StopUnconfirmed;
    snapshot.control.deny = state::DenyReason::RepeatedSequence;
    snapshot.control.owner = state::SourceId::Automation;
    snapshot.control.owner_generation = std::numeric_limits<std::uint32_t>::max();
    snapshot.control.permit = true;
    snapshot.control.recovery_active = true;
    snapshot.control.stop_timeouts = std::numeric_limits<std::uint32_t>::max();
    snapshot.control.escalations = std::numeric_limits<std::uint32_t>::max();
    snapshot.control.ack_mismatches = std::numeric_limits<std::uint32_t>::max();
    snapshot.control.rejected_intents = std::numeric_limits<std::uint32_t>::max();
    snapshot.control.notify_failures = std::numeric_limits<std::uint32_t>::max();
    snapshot.control.emergency_slot_busy = std::numeric_limits<std::uint32_t>::max();
    snapshot.control.delivery_failures[0] = std::numeric_limits<std::uint32_t>::max();
    snapshot.control.delivery_failures[1] = std::numeric_limits<std::uint32_t>::max();

    for (std::size_t i = 0; i < 2; ++i) {
        AxisObservation &axis = snapshot.axis[i];
        axis.meta.valid = true;
        axis.meta.polled_us = snapshot.generated_us;
        axis.status.state = motion::AxisState::Accelerating;
        axis.status.position_step = std::numeric_limits<std::int64_t>::min();
        axis.status.rate_steps_s = -10000.0F;
        axis.status.energized = true;
        axis.status.position_trusted = true;
        axis.status.has_fault = true;
        axis.status.fault_reason = motion::FaultReason::WatchdogTimeout;
        axis.status.late_corrections = std::numeric_limits<std::uint64_t>::max();
        axis.status.late_replans = std::numeric_limits<std::uint64_t>::max();
        axis.status.stale_events = std::numeric_limits<std::uint64_t>::max();
        axis.status.rejected_commands = std::numeric_limits<std::uint64_t>::max();
        axis.read_failures = std::numeric_limits<std::uint32_t>::max();
    }

    snapshot.joystick.meta.valid = true;
    snapshot.joystick.meta.measured_us = snapshot.generated_us;
    snapshot.joystick.calibrated = true;
    snapshot.joystick.output_valid = true;
    snapshot.joystick.neutral = false;
    snapshot.joystick.read_fault = true;
    snapshot.joystick.read_errors = std::numeric_limits<std::uint32_t>::max();

    const Out out(snapshot, sizeof(out.buffer));
    CHECK(out.result == web::SerializeResult::Ok);
    CHECK(structurally_sane(out.text()));
    CHECK(out.written + 1 <= web::kMaxStateJsonBytes);
    std::printf("worst case: %zu bytes of %zu\n", out.written,
                web::kMaxStateJsonBytes);
}

}  // namespace

int main()
{
    testPopulated();
    testEmptySnapshot();
    testNonFiniteRates();
    testRateFormatting();
    testMaxValues();
    testAges();
    testFreshnessBoundary();
    testInvalidatedBlocks();
    testUnknownEnums();
    testControlAgeAndHealth();
    testCapacity();
    testWorstCaseFits();
    return TEST_RESULT();
}
