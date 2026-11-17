#include <cmath>
#include <cstdint>
#include <limits>

#include "calibration/math.hpp"
#include "check.hpp"

using namespace calibration;

namespace {

CalibrationParams params()
{
    CalibrationParams p;
    p.min_revolutions = 1;
    p.max_revolutions = 20;
    p.min_measurements = 2;
    p.max_measurements = 5;
    p.max_spread_ppm = 5000;
    return p;
}

}  // namespace

int main()
{
    CHECK(safeDelta(0, 0) == 0);
    CHECK(safeDelta(10, 3) == 7);
    CHECK(safeDelta(3, 10) == 7);
    CHECK(safeDelta(std::numeric_limits<std::int64_t>::min(),
                    std::numeric_limits<std::int64_t>::max()) ==
          std::numeric_limits<std::uint64_t>::max());

    const CalibrationParams p = params();
    const Measurement m1 = makeMeasurement(0, 1, 1000, 5096,
                                           motion::Direction::Forward, 10, 11, 1, p);
    CHECK(m1.valid);
    CHECK(m1.delta_half_steps == 4096);
    CHECK_NEAR(m1.degrees_per_half_step, 360.0 / 4096.0, 1e-12);
    CHECK_NEAR(m1.half_steps_per_revolution, 4096.0, 1e-9);

    const Measurement zero = makeMeasurement(0, 1, 100, 100,
                                             motion::Direction::Forward, 1, 2, 1, p);
    CHECK(!zero.valid);
    CHECK(zero.reason == InvalidReason::ZeroDelta);

    const Measurement reverse = makeMeasurement(0, 1, 5096, 1000,
                                                motion::Direction::Forward, 1, 2, 1, p);
    CHECK(!reverse.valid);
    CHECK(reverse.reason == InvalidReason::DirectionMismatch);

    const Measurement m2 = makeMeasurement(1, 2, 5096, 13288,
                                           motion::Direction::Forward, 12, 13, 1, p);
    CHECK(m2.valid);

    Measurement list[2] = {m1, m2};
    const CombinedResult combined = combine(list, 2, p);
    CHECK(combined.ok);
    CHECK(combined.sample_count == 2);
    CHECK(combined.total_revolutions == 3);
    CHECK(combined.repeatability == Repeatability::Good);
    CHECK_NEAR(combined.degrees_per_half_step,
               360.0 * 3.0 / (4096.0 + 8192.0), 1e-12);
    CHECK_NEAR(combined.half_steps_per_revolution, 12288.0 / 3.0, 1e-9);

    Measurement different[2] = {m1, m1};
    different[1].degrees_per_half_step *= 1.01;
    different[1].valid = true;
    const CombinedResult bad = combine(different, 2, p);
    CHECK(bad.ok);
    CHECK(bad.repeatability == Repeatability::Warn);
    CHECK_GT(bad.spread_ppm, 5000U);

    CHECK_NEAR(nominalDegreesPerHalfStep(4096), 360.0 / 4096.0, 1e-12);
    CHECK(toNanodegrees(360.0 / 4096.0) == 87890625);

    // Знак исполнительной уставки задаёт только направление.
    CHECK_NEAR(signedRate(motion::Direction::Forward, 120.0F), 120.0, 1e-6);
    CHECK_NEAR(signedRate(motion::Direction::Backward, 120.0F), -120.0, 1e-6);
    CHECK_NEAR(signedRate(motion::Direction::Backward, -120.0F), -120.0, 1e-6);

    return TEST_RESULT();
}
