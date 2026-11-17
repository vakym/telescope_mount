/**
 * @file calibration_json.cpp
 * @brief Сборка JSON статуса калибровки без динамических строк.
 */

#include "web/calibration_json.hpp"

#include <cstdio>
#include <cstring>

namespace web {
namespace {

/// Буфер с проверкой переполнения: частичный документ не отправляется.
struct Writer {
    char *out = nullptr;
    std::size_t capacity = 0;
    std::size_t used = 0;
    bool overflow = false;

    void append(const char *text) noexcept
    {
        if (text == nullptr) {
            overflow = true;
            return;
        }
        const std::size_t length = std::strlen(text);
        if (used + length + 1 > capacity) {
            overflow = true;
            return;
        }
        std::memcpy(out + used, text, length);
        used += length;
        out[used] = '\0';
    }

    void append(char value) noexcept
    {
        if (used + 1 + 1 > capacity) {
            overflow = true;
            return;
        }
        out[used++] = value;
        out[used] = '\0';
    }

    void number(std::uint64_t value, bool negative = false) noexcept
    {
        char buffer[32];
        if (negative) {
            std::snprintf(buffer, sizeof(buffer), "-%llu",
                          static_cast<unsigned long long>(value));
        } else {
            std::snprintf(buffer, sizeof(buffer), "%llu",
                          static_cast<unsigned long long>(value));
        }
        append(buffer);
    }

    void decimal(double value, int precision) noexcept
    {
        char buffer[40];
        std::snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
        append(buffer);
    }

    void key(const char *name) noexcept
    {
        append('"');
        append(name);
        append('"');
        append(':');
    }
};

void writeScale(Writer &writer, motion::AxisId axis, const calibration::AxisScale &scale,
                std::uint32_t nominal) noexcept
{
    writer.append('{');
    writer.key("axis");
    writer.append('"');
    writer.append(axis == motion::AxisId::Azimuth ? "azimuth" : "altitude");
    writer.append('"');
    writer.append(',');
    writer.key("nanodegrees_per_half_step");
    writer.number(scale.nanodegrees_per_half_step < 0
                      ? 0
                      : static_cast<std::uint64_t>(scale.nanodegrees_per_half_step));
    writer.append(',');
    writer.key("degrees_per_half_step");
    writer.decimal(static_cast<double>(scale.nanodegrees_per_half_step) / 1.0e9, 9);
    writer.append(',');
    writer.key("source");
    writer.append('"');
    writer.append(calibration::sourceName(scale.source));
    writer.append('"');
    writer.append(',');
    writer.key("saved");
    writer.append(scale.saved ? "true" : "false");
    writer.append(',');
    writer.key("persistence");
    writer.append('"');
    writer.append(calibration::persistenceName(scale.persistence));
    writer.append('"');
    writer.append(',');
    writer.key("sample_count");
    writer.number(scale.sample_count);
    writer.append(',');
    writer.key("total_revolutions");
    writer.number(scale.total_revolutions);
    writer.append(',');
    writer.key("spread_ppm");
    writer.number(scale.spread_ppm);
    writer.append(',');
    writer.key("nominal_half_steps_per_revolution");
    writer.number(nominal);
    writer.append('}');
}

}  // namespace

CalibrationJsonResult calibrationStatusJson(const CalibrationView &view, char *out,
                                            std::size_t capacity,
                                            std::size_t *written) noexcept
{
    if (out == nullptr || capacity == 0) {
        if (written != nullptr) {
            *written = 0;
        }
        return CalibrationJsonResult::Overflow;
    }
    out[0] = '\0';
    Writer writer{out, capacity, 0, false};

    writer.append('{');
    writer.key("schema");
    writer.number(1);
    writer.append(',');
    writer.key("session");
    writer.append('{');
    writer.key("state");
    writer.append('"');
    writer.append(calibration::sessionStateName(view.state));
    writer.append('"');
    writer.append(',');
    writer.key("error");
    writer.append('"');
    writer.append(calibration::sessionErrorName(view.error));
    writer.append('"');
    writer.append(',');
    writer.key("invalid_reason");
    writer.append('"');
    writer.append(calibration::invalidReasonName(view.invalid_reason));
    writer.append('"');
    writer.append(',');
    writer.key("id");
    writer.number(view.session_id);
    writer.append(',');
    writer.key("revision");
    writer.number(view.revision);
    writer.append(',');
    writer.key("axis");
    writer.append('"');
    writer.append(view.axis == motion::AxisId::Azimuth ? "azimuth" : "altitude");
    writer.append('"');
    writer.append(',');
    writer.key("direction");
    writer.append('"');
    writer.append(view.direction == motion::Direction::Forward ? "forward" : "backward");
    writer.append('"');
    writer.append(',');
    writer.key("revolutions");
    writer.number(view.revolutions);
    writer.append(',');
    writer.key("repeats");
    writer.number(view.repeats);
    writer.append(',');
    writer.key("measurement_count");
    writer.number(view.measurement_count);
    writer.append(',');
    writer.key("mark_recheck_required");
    writer.append(view.mark_recheck_required ? "true" : "false");
    writer.append(',');
    writer.key("hold_active");
    writer.append(view.hold_active ? "true" : "false");
    writer.append(',');
    writer.key("input_ready");
    writer.append(view.input_ready ? "true" : "false");
    writer.append(',');
    writer.key("lease_remaining_ms");
    writer.number(view.lease_remaining_ms);
    writer.append(',');
    writer.key("motion_generation");
    writer.number(view.motion_generation);
    writer.append(',');
    writer.key("result");
    writer.append('{');
    writer.key("ok");
    writer.append(view.result.ok ? "true" : "false");
    writer.append(',');
    writer.key("degrees_per_half_step");
    writer.decimal(view.result.ok ? view.result.degrees_per_half_step : 0.0, 9);
    writer.append(',');
    writer.key("half_steps_per_revolution");
    writer.decimal(view.result.ok ? view.result.half_steps_per_revolution : 0.0, 6);
    writer.append(',');
    writer.key("sample_count");
    writer.number(view.result.sample_count);
    writer.append(',');
    writer.key("total_revolutions");
    writer.number(view.result.total_revolutions);
    writer.append(',');
    writer.key("spread_ppm");
    writer.number(view.result.spread_ppm);
    writer.append(',');
    writer.key("max_deviation_ppm");
    writer.number(view.result.max_deviation_ppm);
    writer.append(',');
    writer.key("repeatability");
    writer.append('"');
    writer.append(calibration::repeatabilityName(view.result.repeatability));
    writer.append('"');
    writer.append('}');
    writer.append('}');
    writer.append(',');
    writer.key("flash_window_active");
    writer.append(view.flash_window_active ? "true" : "false");
    writer.append(',');
    writer.key("axes");
    writer.append('[');
    writeScale(writer, motion::AxisId::Azimuth, view.scale[0],
               view.nominal_half_steps_per_revolution[0]);
    writer.append(',');
    writeScale(writer, motion::AxisId::Altitude, view.scale[1],
               view.nominal_half_steps_per_revolution[1]);
    writer.append(']');
    writer.append('}');

    if (writer.overflow) {
        out[0] = '\0';
        if (written != nullptr) {
            *written = 0;
        }
        return CalibrationJsonResult::Overflow;
    }
    if (written != nullptr) {
        *written = writer.used;
    }
    return CalibrationJsonResult::Ok;
}

}  // namespace web
