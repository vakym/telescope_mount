#include "calibration/math.hpp"

#include <cmath>

#include <cmath>
#include <limits>

namespace calibration {
namespace {

int directionSign(motion::Direction direction) noexcept
{
    return direction == motion::Direction::Forward ? 1 : -1;
}

}  // namespace

std::uint64_t safeDelta(std::int64_t a, std::int64_t b) noexcept
{
    if (a == b) {
        return 0;
    }
    const std::uint64_t ua = static_cast<std::uint64_t>(a);
    const std::uint64_t ub = static_cast<std::uint64_t>(b);
    return a > b ? ua - ub : ub - ua;
}

Measurement makeMeasurement(std::uint32_t index, std::uint32_t revolutions,
                            std::int64_t start_position,
                            std::int64_t end_position,
                            motion::Direction expected_direction,
                            std::uint64_t start_ticket,
                            std::uint64_t stop_ticket,
                            std::uint32_t epoch,
                            const CalibrationParams &params) noexcept
{
    Measurement out{};
    out.index = index;
    out.revolutions = revolutions;
    out.start_position = start_position;
    out.end_position = end_position;
    out.start_ticket = start_ticket;
    out.stop_ticket = stop_ticket;
    out.epoch = epoch;
    out.delta_half_steps = safeDelta(start_position, end_position);
    out.valid = false;
    out.reason = InvalidReason::InvalidResult;

    if (revolutions == 0) {
        out.reason = InvalidReason::ZeroDelta;
        return out;
    }
    if (out.delta_half_steps == 0) {
        out.reason = InvalidReason::ZeroDelta;
        return out;
    }

    const int actual_sign = end_position > start_position ? 1
                            : end_position < start_position ? -1
                                                            : 0;
    if (actual_sign != directionSign(expected_direction)) {
        out.reason = InvalidReason::DirectionMismatch;
        return out;
    }

    const double delta = static_cast<double>(out.delta_half_steps);
    const double revs = static_cast<double>(revolutions);
    const double k = 360.0 * revs / delta;
    const double s = delta / revs;
    out.degrees_per_half_step = k;
    out.half_steps_per_revolution = s;

    if (!std::isfinite(k) || !std::isfinite(s) || k <= 0.0 || s <= 0.0) {
        out.reason = InvalidReason::InvalidResult;
        return out;
    }
    if (k < params.min_degrees_per_half_step ||
        k > params.max_degrees_per_half_step) {
        out.reason = InvalidReason::InvalidResult;
        return out;
    }

    out.valid = true;
    out.reason = InvalidReason::Ok;
    return out;
}

CombinedResult combine(const Measurement *measurements, std::size_t count,
                       const CalibrationParams &params) noexcept
{
    CombinedResult out{};
    if (measurements == nullptr || count == 0) {
        out.reason = InvalidReason::InvalidResult;
        return out;
    }

    long double total_delta = 0.0L;
    std::uint64_t total_revolutions = 0;
    std::uint32_t valid_count = 0;
    double min_k = std::numeric_limits<double>::infinity();
    double max_k = 0.0;

    for (std::size_t i = 0; i < count; ++i) {
        const Measurement &m = measurements[i];
        if (!m.valid || m.revolutions == 0 || m.delta_half_steps == 0) {
            continue;
        }
        const double k = m.degrees_per_half_step;
        if (!std::isfinite(k) || k <= 0.0) {
            continue;
        }
        ++valid_count;
        total_delta += static_cast<long double>(m.delta_half_steps);
        total_revolutions += m.revolutions;
        if (k < min_k) {
            min_k = k;
        }
        if (k > max_k) {
            max_k = k;
        }
    }

    out.sample_count = valid_count;
    out.total_revolutions = total_revolutions;

    if (valid_count == 0 || total_delta <= 0.0L || total_revolutions == 0) {
        out.reason = InvalidReason::InvalidResult;
        out.repeatability = Repeatability::Invalid;
        return out;
    }

    const long double k = 360.0L * static_cast<long double>(total_revolutions) /
                          total_delta;
    const long double s = total_delta /
                          static_cast<long double>(total_revolutions);
    const double kd = static_cast<double>(k);
    const double sd = static_cast<double>(s);
    if (!std::isfinite(kd) || !std::isfinite(sd) || kd <= 0.0 || sd <= 0.0 ||
        kd < params.min_degrees_per_half_step ||
        kd > params.max_degrees_per_half_step) {
        out.reason = InvalidReason::InvalidResult;
        out.repeatability = Repeatability::Invalid;
        return out;
    }

    out.degrees_per_half_step = kd;
    out.half_steps_per_revolution = sd;

    std::uint32_t max_deviation = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const Measurement &m = measurements[i];
        if (!m.valid || m.degrees_per_half_step <= 0.0) {
            continue;
        }
        const double ratio = m.degrees_per_half_step / kd - 1.0;
        const double ppm = std::fabs(ratio) * 1'000'000.0;
        const std::uint32_t value = static_cast<std::uint32_t>(ppm + 0.5);
        if (value > max_deviation) {
            max_deviation = value;
        }
    }
    out.max_deviation_ppm = max_deviation;

    if (valid_count < params.min_measurements) {
        out.ok = true;
        out.reason = InvalidReason::Ok;
        out.repeatability = Repeatability::Insufficient;
        return out;
    }

    if (max_k > min_k) {
        const double ppm = (max_k - min_k) / kd * 1'000'000.0;
        out.spread_ppm = static_cast<std::uint32_t>(ppm + 0.5);
    }

    out.ok = true;
    out.reason = InvalidReason::Ok;
    out.repeatability = out.spread_ppm > params.max_spread_ppm
                            ? Repeatability::Warn
                            : Repeatability::Good;
    return out;
}

double nominalDegreesPerHalfStep(std::uint32_t half_steps_per_revolution) noexcept
{
    if (half_steps_per_revolution == 0) {
        return 0.0;
    }
    return 360.0 / static_cast<double>(half_steps_per_revolution);
}

std::int64_t toNanodegrees(double degrees_per_half_step) noexcept
{
    if (!std::isfinite(degrees_per_half_step)) {
        return 0;
    }
    const long double value =
        static_cast<long double>(degrees_per_half_step) * 1'000'000'000.0L;
    if (value >= static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
        return std::numeric_limits<std::int64_t>::max();
    }
    if (value <= static_cast<long double>(std::numeric_limits<std::int64_t>::min())) {
        return std::numeric_limits<std::int64_t>::min();
    }
    return static_cast<std::int64_t>(value >= 0.0L ? value + 0.5L : value - 0.5L);
}

float signedRate(motion::Direction direction, float magnitude) noexcept
{
    if (!std::isfinite(magnitude)) {
        return 0.0F;
    }
    const float absolute = std::fabs(magnitude);
    return direction == motion::Direction::Forward ? absolute : -absolute;
}

bool movePermissionValid(bool move_active, Micros now, Micros until,
                         Micros min_lease) noexcept
{
    if (!move_active || until <= now) {
        return false;
    }
    return (until - now) > min_lease;
}

}  // namespace calibration
