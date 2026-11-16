/**
 * @file state_json.cpp
 * @brief Ограниченный писатель JSON без динамических выделений.
 *
 * Писатель не бросает исключений и не использует printf: все числа
 * форматируются целочисленной арифметикой, поэтому результат не зависит от
 * поддержки `%f` в newlib и от локали.
 */

#include "web/state_json.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "state/channels.hpp"
#include "state/health.hpp"

namespace web {
namespace {

/// Возраст блока по времени источника.
struct AgeInfo {
    bool known = false;
    std::uint64_t ms = 0;
    bool fresh = false;
};

AgeInfo ageInfo(state::Micros now_us, state::Micros then_us,
                state::Micros limit_us) noexcept
{
    if (then_us == state::kNever || now_us < then_us) {
        return AgeInfo{};
    }
    const state::Micros delta = now_us - then_us;
    AgeInfo info;
    info.known = true;
    info.ms = static_cast<std::uint64_t>(delta / 1000U);
    info.fresh = delta <= limit_us;
    return info;
}

/// Ограниченный писатель: любая нехватка ёмкости делает документ негодным.
class Writer {
public:
    Writer(char *buffer, std::size_t capacity) noexcept
        : buffer_(buffer), capacity_(capacity)
    {
        if (buffer_ != nullptr && capacity_ > 0) {
            buffer_[0] = '\0';
        }
    }

    bool ok() const noexcept { return ok_; }
    std::size_t size() const noexcept { return length_; }

    void ch(char value) noexcept { put(value); }

    void plain(const char *text) noexcept
    {
        if (text == nullptr) {
            return;
        }
        while (*text != '\0' && ok_) {
            put(*text++);
        }
    }

    void quoted(const char *text) noexcept
    {
        ch('"');
        plain(text);
        ch('"');
    }

    void nullValue() noexcept { plain("null"); }

    void boolean(bool value) noexcept { plain(value ? "true" : "false"); }

    void unsignedNumber(std::uint64_t value) noexcept
    {
        char digits[20];
        std::size_t count = 0;
        do {
            digits[count++] = static_cast<char>('0' + (value % 10U));
            value /= 10U;
        } while (value != 0U && count < sizeof(digits));
        while (count > 0 && ok_) {
            put(digits[--count]);
        }
    }

    void unsignedString(std::uint64_t value) noexcept
    {
        ch('"');
        unsignedNumber(value);
        ch('"');
    }

    void signedString(std::int64_t value) noexcept
    {
        ch('"');
        if (value < 0) {
            ch('-');
            const std::uint64_t magnitude =
                static_cast<std::uint64_t>(0) - static_cast<std::uint64_t>(value);
            unsignedNumber(magnitude);
        } else {
            unsignedNumber(static_cast<std::uint64_t>(value));
        }
        ch('"');
    }

    /**
     * @brief Значение с одним десятичным знаком.
     *
     * Нефинитное значение и значение вне допуска дают `null`; -0.0
     * печатается как 0.0, знак отрицательных значений сохраняется.
     */
    void fixed1(float value) noexcept
    {
        if (!std::isfinite(value)) {
            nullValue();
            return;
        }
        // NaN не проходит ни одно из сравнений, поэтому проверка полная.
        if (!(value >= -kMaxRateAbsHalfStepsS && value <= kMaxRateAbsHalfStepsS)) {
            nullValue();
            return;
        }
        const bool negative = value < 0.0F;
        const float magnitude = negative ? -value : value;
        const std::int32_t units = static_cast<std::int32_t>(magnitude * 10.0F + 0.5F);
        const std::int32_t scaled = negative ? -units : units;
        const bool is_negative = scaled < 0;
        const std::uint32_t abs_units =
            static_cast<std::uint32_t>(is_negative ? -scaled : scaled);
        if (is_negative) {
            ch('-');
        }
        unsignedNumber(abs_units / 10U);
        ch('.');
        ch(static_cast<char>('0' + (abs_units % 10U)));
    }

private:
    void put(char value) noexcept
    {
        if (!ok_) {
            return;
        }
        if (buffer_ == nullptr || capacity_ == 0) {
            ok_ = false;
            return;
        }
        // Место под завершающий нуль сохраняется всегда.
        if (length_ + 1U >= capacity_) {
            ok_ = false;
            return;
        }
        buffer_[length_++] = value;
        buffer_[length_] = '\0';
    }

    char *buffer_ = nullptr;
    std::size_t capacity_ = 0;
    std::size_t length_ = 0;
    bool ok_ = true;
};

void beginObject(Writer &writer) noexcept
{
    writer.ch('{');
}

void endObject(Writer &writer) noexcept
{
    writer.ch('}');
}

void key(Writer &writer, bool &first, const char *name) noexcept
{
    if (!first) {
        writer.ch(',');
    }
    first = false;
    writer.quoted(name);
    writer.ch(':');
}

void writeAge(Writer &writer, const AgeInfo &age) noexcept
{
    if (age.known) {
        writer.unsignedNumber(age.ms);
    } else {
        writer.nullValue();
    }
}

void writeNullFields(Writer &writer, bool &first,
                     const char *const *names, std::size_t count) noexcept
{
    for (std::size_t i = 0; i < count; ++i) {
        key(writer, first, names[i]);
        writer.nullValue();
    }
}

constexpr const char *kAxisNullableFields[] = {
    "state",           "position_half_steps", "rate_half_steps_s",
    "energized",       "trusted",             "fault",
    "fault_reason",    "read_failures",       "late_corrections",
    "late_replans",    "stale_events",        "rejected_commands",
};

constexpr const char *kJoystickNullableFields[] = {
    "calibrated", "output_valid", "neutral", "fault", "read_errors",
};

constexpr const char *kControlNullableFields[] = {
    "phase", "latch", "deny", "permit", "owner", "generation", "recovery",
};

void writeAxis(Writer &writer, const state::AxisObservation &obs,
               state::Micros now_us, state::Micros stale_us) noexcept
{
    // health описывает состояние (fault/disabled важнее возраста), а
    // freshness — только возраст данных. Это разные признаки, и оба нужны.
    const state::AxisHealth health = state::axisHealth(obs, now_us, stale_us);
    const state::Freshness fresh_state =
        obs.meta.valid
            ? state::freshness(now_us, obs.meta.polled_us, stale_us)
            : state::Freshness::Never;
    const AgeInfo age =
        obs.meta.valid ? ageInfo(now_us, obs.meta.polled_us, stale_us)
                       : AgeInfo{};
    bool first = true;
    beginObject(writer);
    key(writer, first, "valid");
    writer.boolean(obs.meta.valid);
    key(writer, first, "health");
    writer.quoted(state::axisHealthName(health));
    key(writer, first, "freshness");
    writer.quoted(state::freshnessName(fresh_state));
    key(writer, first, "age_ms");
    writeAge(writer, age);

    if (!obs.meta.valid) {
        writeNullFields(writer, first, kAxisNullableFields,
                        sizeof(kAxisNullableFields) / sizeof(kAxisNullableFields[0]));
    } else {
        key(writer, first, "state");
        writer.quoted(state::axisStateName(obs.status.state));
        key(writer, first, "position_half_steps");
        writer.signedString(obs.status.position_step);
        key(writer, first, "rate_half_steps_s");
        writer.fixed1(obs.status.rate_steps_s);
        key(writer, first, "energized");
        writer.boolean(obs.status.energized);
        key(writer, first, "trusted");
        writer.boolean(obs.status.position_trusted);
        key(writer, first, "fault");
        writer.boolean(obs.status.has_fault);
        key(writer, first, "fault_reason");
        if (obs.status.has_fault) {
            writer.quoted(state::faultReasonName(obs.status.fault_reason));
        } else {
            writer.nullValue();
        }
        key(writer, first, "read_failures");
        writer.unsignedNumber(obs.read_failures);
        key(writer, first, "late_corrections");
        writer.unsignedString(obs.status.late_corrections);
        key(writer, first, "late_replans");
        writer.unsignedString(obs.status.late_replans);
        key(writer, first, "stale_events");
        writer.unsignedString(obs.status.stale_events);
        key(writer, first, "rejected_commands");
        writer.unsignedString(obs.status.rejected_commands);
    }
    endObject(writer);
}

void writeJoystick(Writer &writer, const state::JoystickObservation &obs,
                   state::Micros now_us, state::Micros stale_us) noexcept
{
    const state::JoyHealth health = state::joyHealth(obs, now_us, stale_us);
    const state::Freshness fresh_state =
        obs.meta.valid
            ? state::freshness(now_us, obs.meta.measured_us, stale_us)
            : state::Freshness::Never;
    const AgeInfo age =
        obs.meta.valid ? ageInfo(now_us, obs.meta.measured_us, stale_us)
                       : AgeInfo{};
    bool first = true;
    beginObject(writer);
    key(writer, first, "valid");
    writer.boolean(obs.meta.valid);
    key(writer, first, "health");
    writer.quoted(state::joyHealthName(health));
    key(writer, first, "freshness");
    writer.quoted(state::freshnessName(fresh_state));
    key(writer, first, "age_ms");
    writeAge(writer, age);

    if (!obs.meta.valid) {
        writeNullFields(writer, first, kJoystickNullableFields,
                        sizeof(kJoystickNullableFields) /
                            sizeof(kJoystickNullableFields[0]));
    } else {
        key(writer, first, "calibrated");
        writer.boolean(obs.calibrated);
        key(writer, first, "output_valid");
        writer.boolean(obs.output_valid);
        key(writer, first, "neutral");
        writer.boolean(obs.neutral);
        key(writer, first, "fault");
        writer.boolean(obs.read_fault);
        key(writer, first, "read_errors");
        writer.unsignedNumber(obs.read_errors);
    }
    endObject(writer);
}

void writeControl(Writer &writer, const state::ControlObservation &obs,
                  state::Micros now_us, state::Micros stale_us) noexcept
{
    const AgeInfo age =
        obs.meta.valid ? ageInfo(now_us, obs.meta.polled_us, stale_us)
                       : AgeInfo{};
    const state::Freshness fresh_state =
        obs.meta.valid
            ? state::freshness(now_us, obs.meta.polled_us, stale_us)
            : state::Freshness::Never;
    const char *health = "unknown";
    if (obs.meta.valid && age.known) {
        health = age.fresh ? "ok" : "stale";
    }

    bool first = true;
    beginObject(writer);
    key(writer, first, "valid");
    writer.boolean(obs.meta.valid);
    key(writer, first, "health");
    writer.quoted(health);
    key(writer, first, "freshness");
    writer.quoted(state::freshnessName(fresh_state));
    key(writer, first, "age_ms");
    writeAge(writer, age);

    if (!obs.meta.valid) {
        writeNullFields(writer, first, kControlNullableFields,
                        sizeof(kControlNullableFields) /
                            sizeof(kControlNullableFields[0]));
    } else {
        key(writer, first, "phase");
        writer.quoted(state::phaseName(obs.phase));
        key(writer, first, "latch");
        writer.quoted(state::latchName(obs.latch));
        key(writer, first, "deny");
        writer.quoted(state::denyName(obs.deny));
        key(writer, first, "permit");
        writer.boolean(obs.permit);
        key(writer, first, "owner");
        writer.quoted(state::sourceName(obs.owner));
        key(writer, first, "generation");
        writer.unsignedNumber(obs.owner_generation);
        key(writer, first, "recovery");
        writer.boolean(obs.recovery_active);
    }
    endObject(writer);
}

void writeDiag(Writer &writer, const state::ControlObservation &obs) noexcept
{
    bool first = true;
    beginObject(writer);
    if (!obs.meta.valid) {
        constexpr const char *kNames[] = {
            "stop_timeouts", "escalations", "ack_mismatches",
            "delivery_failures", "rejected_intents", "notify_failures",
            "emergency_slot_busy",
        };
        writeNullFields(writer, first, kNames, sizeof(kNames) / sizeof(kNames[0]));
    } else {
        key(writer, first, "stop_timeouts");
        writer.unsignedNumber(obs.stop_timeouts);
        key(writer, first, "escalations");
        writer.unsignedNumber(obs.escalations);
        key(writer, first, "ack_mismatches");
        writer.unsignedNumber(obs.ack_mismatches);
        key(writer, first, "delivery_failures");
        writer.ch('[');
        writer.unsignedNumber(obs.delivery_failures[0]);
        writer.ch(',');
        writer.unsignedNumber(obs.delivery_failures[1]);
        writer.ch(']');
        key(writer, first, "rejected_intents");
        writer.unsignedNumber(obs.rejected_intents);
        key(writer, first, "notify_failures");
        writer.unsignedNumber(obs.notify_failures);
        key(writer, first, "emergency_slot_busy");
        writer.unsignedNumber(obs.emergency_slot_busy);
    }
    endObject(writer);
}

}  // namespace

SerializeResult stateJson(const state::SystemSnapshot &snapshot,
                          const SerializeOptions &options, char *out,
                          std::size_t capacity,
                          std::size_t *written) noexcept
{
    if (written != nullptr) {
        *written = 0;
    }
    if (out == nullptr || capacity == 0) {
        return SerializeResult::Overflow;
    }

    Writer writer(out, capacity);
    const state::Micros now_us = snapshot.generated_us;

    bool first = true;
    writer.ch('{');
    key(writer, first, "schema");
    writer.unsignedNumber(kSchemaVersion);
    key(writer, first, "generated_ms");
    writer.unsignedNumber(now_us / 1000U);

    key(writer, first, "control");
    writeControl(writer, snapshot.control, now_us, options.control_stale_us);

    key(writer, first, "axes");
    writer.ch('{');
    bool axis_first = true;
    key(writer, axis_first, "azimuth");
    writeAxis(writer, snapshot.axis[0], now_us, options.axis_stale_us);
    key(writer, axis_first, "altitude");
    writeAxis(writer, snapshot.axis[1], now_us, options.axis_stale_us);
    writer.ch('}');

    key(writer, first, "joystick");
    writeJoystick(writer, snapshot.joystick, now_us, options.joystick_stale_us);

    key(writer, first, "diag");
    writeDiag(writer, snapshot.control);

    writer.ch('}');

    if (!writer.ok()) {
        out[0] = '\0';
        return SerializeResult::Overflow;
    }
    if (written != nullptr) {
        *written = writer.size();
    }
    return SerializeResult::Ok;
}

}  // namespace web
