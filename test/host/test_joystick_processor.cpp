/**
 * @file test_joystick_processor.cpp
 * @brief Проверки калибровки, фильтрации, нормализации и мёртвой зоны.
 */

#include <cmath>
#include <cstdint>
#include <limits>

#include "check.hpp"
#include "input/joystick_processor.hpp"

namespace {

using input::RawSample;
using input::StickAxis;

RawSample sample(std::uint16_t x, std::uint16_t y)
{
    RawSample value;
    value.counts[0] = x;
    value.counts[1] = y;
    value.valid = true;
    return value;
}

RawSample invalidSample(std::uint16_t x, std::uint16_t y)
{
    RawSample value = sample(x, y);
    value.valid = false;
    return value;
}

input::JoystickConfig baseConfig()
{
    input::JoystickConfig config;
    config.filter_alpha = 1.0F;  // без сглаживания: тесты проверяют алгоритм
    return config;
}

std::uint16_t countsFor(float normalized, const input::AxisCalibration &cal)
{
    const float scale = normalized >= 0.0F ? (cal.max - cal.center)
                                           : (cal.center - cal.min);
    const float value = cal.center + normalized * scale;
    return static_cast<std::uint16_t>(std::lround(value));
}

RawSample normalizedSample(const input::JoystickConfig &config, float x, float y)
{
    return sample(countsFor(x, config.axis[0]), countsFor(y, config.axis[1]));
}

void calibrationStable()
{
    input::CenterCalibrator calibrator;
    CHECK(calibrator.configure(4, 10.0F, 1000.0F, 3000.0F));
    for (int i = 0; i < 4; ++i) {
        CHECK(calibrator.add(sample(2002, 2100)));
    }
    CHECK(calibrator.ready());
    CHECK(calibrator.stable());
    CHECK_NEAR(calibrator.center(StickAxis::X), 2002.0, 0.5);
    CHECK_NEAR(calibrator.center(StickAxis::Y), 2100.0, 0.5);
}

void calibrationRejectsNoise()
{
    input::CenterCalibrator calibrator;
    CHECK(calibrator.configure(4, 10.0F, 1000.0F, 3000.0F));
    CHECK(calibrator.add(sample(2000, 2000)));
    CHECK(calibrator.add(sample(2020, 2000)));
    CHECK(calibrator.add(sample(1990, 2000)));
    CHECK(calibrator.add(sample(2005, 2000)));
    CHECK(calibrator.ready());
    CHECK(!calibrator.stable());
}

void calibrationRejectsOutOfBounds()
{
    input::CenterCalibrator calibrator;
    CHECK(calibrator.configure(4, 1000.0F, 1000.0F, 3000.0F));
    for (int i = 0; i < 4; ++i) {
        CHECK(calibrator.add(sample(3100, 3100)));
    }
    CHECK(calibrator.ready());
    CHECK(!calibrator.stable());
}

void calibrationAcceptsStableOffCenter()
{
    // Известное ограничение: неподвижное отклонение внутри коридора
    // неотличимо от центра.
    input::CenterCalibrator calibrator;
    CHECK(calibrator.configure(4, 10.0F, 1000.0F, 3000.0F));
    for (int i = 0; i < 4; ++i) {
        CHECK(calibrator.add(sample(2600, 2600)));
    }
    CHECK(calibrator.ready());
    CHECK(calibrator.stable());
}

void calibrationRejectsInvalidConfig()
{
    using limits = std::numeric_limits<float>;
    input::CenterCalibrator calibrator;
    CHECK(!calibrator.configure(0, 10.0F, 1000.0F, 3000.0F));
    CHECK(!calibrator.configure(4, limits::quiet_NaN(), 1000.0F, 3000.0F));
    CHECK(!calibrator.configure(4, -1.0F, 1000.0F, 3000.0F));
    CHECK(!calibrator.configure(4, 10.0F, 3000.0F, 1000.0F));
    CHECK(!calibrator.configure(4, 10.0F, limits::quiet_NaN(), 3000.0F));
    CHECK(!calibrator.add(sample(2000, 2000)));
}

void invalidSampleDoesNotChangeState()
{
    input::JoystickProcessor processor;
    const auto config = baseConfig();
    CHECK(processor.configure(config));
    const auto before = processor.update(sample(2048, 2048));
    const auto after = processor.update(invalidSample(4095, 4095));
    CHECK_NEAR(after.filtered[0], before.filtered[0], 0.001);
    CHECK_NEAR(after.filtered[1], before.filtered[1], 0.001);
    CHECK_NEAR(after.normalized[0], before.normalized[0], 0.001);
    CHECK(after.neutral);
}

void centerIsNeutral()
{
    input::JoystickProcessor processor;
    const auto config = baseConfig();
    CHECK(processor.configure(config));
    const auto state = processor.update(sample(2048, 2048));
    CHECK(state.neutral);
    CHECK_NEAR(state.normalized[0], 0.0, 0.001);
    CHECK_NEAR(state.normalized[1], 0.0, 0.001);
    CHECK_NEAR(state.magnitude, 0.0, 0.001);
}

void deadZone()
{
    input::JoystickProcessor processor;
    const auto config = baseConfig();
    CHECK(processor.configure(config));

    const auto inside = processor.update(normalizedSample(config, 0.05F, 0.0F));
    CHECK(inside.neutral);

    const auto outside = processor.update(normalizedSample(config, 0.20F, 0.0F));
    CHECK(!outside.neutral);
}

void hysteresis()
{
    input::JoystickProcessor processor;
    const auto config = baseConfig();
    CHECK(processor.configure(config));

    const auto active = processor.update(normalizedSample(config, 0.30F, 0.0F));
    CHECK(!active.neutral);

    // 0.11 между dead_zone_exit (0.10) и dead_zone (0.12): остаётся активным.
    const auto middle = processor.update(normalizedSample(config, 0.11F, 0.0F));
    CHECK(!middle.neutral);

    const auto neutral = processor.update(normalizedSample(config, 0.05F, 0.0F));
    CHECK(neutral.neutral);
}

void zeroExitThreshold()
{
    input::JoystickProcessor processor;
    auto config = baseConfig();
    config.dead_zone_exit = 0.0F;
    CHECK(processor.configure(config));

    CHECK(!processor.update(normalizedSample(config, 0.30F, 0.0F)).neutral);
    // magnitude == 0 и порог выхода 0: проверка <= возвращает нейтраль.
    CHECK(processor.update(sample(2048, 2048)).neutral);
}

void fullDeflection()
{
    input::JoystickProcessor processor;
    const auto config = baseConfig();
    CHECK(processor.configure(config));

    const auto state = processor.update(sample(4095, 2048));
    CHECK_NEAR(state.normalized[0], 1.0, 0.001);
    CHECK_NEAR(state.magnitude, 1.0, 0.001);
    CHECK(!state.neutral);
}

void invertedAxis()
{
    input::JoystickProcessor processor;
    auto config = baseConfig();
    config.axis[0].invert = true;
    CHECK(processor.configure(config));

    const auto state = processor.update(sample(4095, 2048));
    CHECK_NEAR(state.normalized[0], -1.0, 0.001);
    CHECK_NEAR(state.magnitude, 1.0, 0.001);
}

void diagonalMagnitude()
{
    input::JoystickProcessor processor;
    const auto config = baseConfig();
    CHECK(processor.configure(config));

    const auto state = processor.update(sample(4095, 4095));
    CHECK_NEAR(state.normalized[0], 1.0, 0.001);
    CHECK_NEAR(state.normalized[1], 1.0, 0.001);
    // Длина вектора sqrt(2), а magnitude ограничена единицей.
    CHECK_NEAR(state.normalized[0] * state.normalized[0] +
                   state.normalized[1] * state.normalized[1],
               2.0, 0.01);
    CHECK_NEAR(state.magnitude, 1.0, 0.001);
}

void clampsBeyondEdges()
{
    input::JoystickProcessor processor;
    auto config = baseConfig();
    config.axis[0].center = 2500.0F;
    config.axis[0].max = 3000.0F;
    CHECK(processor.configure(config));

    const auto state = processor.update(sample(4095, 2048));
    CHECK_NEAR(state.normalized[0], 1.0, 0.001);
}

void invalidConfig()
{
    using limits = std::numeric_limits<float>;
    input::JoystickProcessor processor;
    auto config = baseConfig();
    CHECK(input::isValid(config));

    config = baseConfig();
    config.dead_zone = limits::quiet_NaN();
    CHECK(!input::isValid(config));
    CHECK(!processor.configure(config));

    config = baseConfig();
    config.dead_zone_exit = limits::quiet_NaN();
    CHECK(!input::isValid(config));

    config = baseConfig();
    config.filter_alpha = limits::quiet_NaN();
    CHECK(!input::isValid(config));

    config = baseConfig();
    config.dead_zone = 0.0F;
    CHECK(!input::isValid(config));

    config = baseConfig();
    config.dead_zone_exit = config.dead_zone;
    CHECK(!input::isValid(config));

    config = baseConfig();
    config.axis[0].center = config.axis[0].max;
    CHECK(!input::isValid(config));
}

void emaTransition()
{
    input::JoystickProcessor processor;
    auto config = baseConfig();
    config.filter_alpha = 0.25F;
    CHECK(processor.configure(config));

    // filtered1 = 0.25 * 3048 + 0.75 * 2048 = 2298
    auto state = processor.update(sample(3048, 2048));
    CHECK_NEAR(state.filtered[0], 2298.0, 0.5);

    // filtered2 = 0.25 * 3048 + 0.75 * 2298 = 2485.5
    state = processor.update(sample(3048, 2048));
    CHECK_NEAR(state.filtered[0], 2485.5, 0.5);

    const float before = state.filtered[0];
    state = processor.update(invalidSample(0, 4095));
    CHECK_NEAR(state.filtered[0], before, 0.001);
}

void resetBehaviour()
{
    input::JoystickProcessor processor;
    const auto config = baseConfig();
    CHECK(processor.configure(config));

    CHECK(!processor.update(normalizedSample(config, 0.50F, 0.0F)).neutral);
    processor.reset();
    const auto reset = processor.state();
    CHECK(reset.neutral);
    CHECK_NEAR(reset.filtered[0], config.axis[0].center, 0.001);
    CHECK_NEAR(reset.normalized[0], 0.0, 0.001);

    CHECK(!processor.update(normalizedSample(config, 0.50F, 0.0F)).neutral);
    // Сброс по свежему отсчёту: 0.11 находится между порогами, но история
    // гистерезиса сброшена в нейтраль, поэтому состояние нейтральное.
    const auto between = processor.resetWithSample(normalizedSample(config, 0.11F, 0.0F));
    CHECK(between.neutral);

    const auto above = processor.resetWithSample(normalizedSample(config, 0.13F, 0.0F));
    CHECK(!above.neutral);
}

void asymmetricCenter()
{
    input::JoystickProcessor processor;
    auto config = baseConfig();
    config.axis[0].center = 1900.0F;
    CHECK(processor.configure(config));

    const auto high = processor.update(sample(4095, 2048));
    CHECK_NEAR(high.normalized[0], 1.0, 0.001);

    const auto low = processor.update(sample(0, 2048));
    CHECK_NEAR(low.normalized[0], -1.0, 0.001);
}

}  // namespace

int main()
{
    calibrationStable();
    calibrationRejectsNoise();
    calibrationRejectsOutOfBounds();
    calibrationAcceptsStableOffCenter();
    calibrationRejectsInvalidConfig();
    invalidSampleDoesNotChangeState();
    centerIsNeutral();
    deadZone();
    hysteresis();
    zeroExitThreshold();
    fullDeflection();
    invertedAxis();
    diagonalMagnitude();
    clampsBeyondEdges();
    invalidConfig();
    emaTransition();
    resetBehaviour();
    asymmetricCenter();
    return TEST_RESULT();
}
