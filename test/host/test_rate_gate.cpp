/**
 * @file test_rate_gate.cpp
 * @brief Дедупликация уставок: нуль вытесняет ожидающую, дубли не теряются.
 */

#include <initializer_list>

#include "check.hpp"
#include "control/rate_gate.hpp"

namespace {

using motion::AxisId;

constexpr std::uint32_t kGen = 1;

void firstValueIsSent()
{
    control::RateGate gate;
    gate.configure({8.0F});
    const auto decision = gate.evaluate(AxisId::Azimuth, 0.0F, kGen);
    CHECK(decision.send);
    CHECK_NEAR(decision.value, 0.0, 1e-6);
}

void duplicateIsSuppressed()
{
    control::RateGate gate;
    gate.configure({8.0F});
    auto first = gate.evaluate(AxisId::Azimuth, 100.0F, kGen);
    CHECK(first.send);
    gate.noteEnqueued(AxisId::Azimuth, first.value, kGen);

    CHECK(!gate.evaluate(AxisId::Azimuth, 100.0F, kGen).send);
    CHECK(!gate.evaluate(AxisId::Azimuth, 104.0F, kGen).send);
    CHECK(gate.evaluate(AxisId::Azimuth, 108.0F, kGen).send);
}

void zeroWinsOverPending()
{
    control::RateGate gate;
    gate.configure({8.0F});
    const auto positive = gate.evaluate(AxisId::Altitude, 200.0F, kGen);
    CHECK(positive.send);
    gate.noteFailed(AxisId::Altitude);

    // Нуль вытесняет ожидающую ненулевую уставку.
    const auto zero = gate.evaluate(AxisId::Altitude, 0.0F, kGen);
    CHECK(zero.send);
    CHECK_NEAR(zero.value, 0.0, 1e-6);
}

void pendingRepeatsCurrentValue()
{
    control::RateGate gate;
    gate.configure({8.0F});
    const auto first = gate.evaluate(AxisId::Azimuth, 300.0F, kGen);
    CHECK(first.send);
    gate.noteFailed(AxisId::Azimuth);

    // Повторяем АКТУАЛЬНОЕ значение, а не прежнее.
    const auto repeat = gate.evaluate(AxisId::Azimuth, 305.0F, kGen);
    CHECK(repeat.send);
    CHECK_NEAR(repeat.value, 305.0, 1e-3);
}

void signChangeIsSent()
{
    control::RateGate gate;
    gate.configure({8.0F});
    const auto first = gate.evaluate(AxisId::Azimuth, 100.0F, kGen);
    gate.noteEnqueued(AxisId::Azimuth, first.value, kGen);
    const auto reversed = gate.evaluate(AxisId::Azimuth, -100.0F, kGen);
    CHECK(reversed.send);
    CHECK(reversed.value < 0.0F);
}

void generationResetsBothAxes()
{
    control::RateGate gate;
    gate.configure({8.0F});

    // Поколение A: обе оси успешно поставлены.
    for (const auto id : {AxisId::Azimuth, AxisId::Altitude}) {
        const auto decision = gate.evaluate(id, 100.0F, kGen);
        CHECK(decision.send);
        gate.noteEnqueued(id, decision.value, kGen);
    }
    CHECK(!gate.evaluate(AxisId::Azimuth, 100.0F, kGen).send);
    CHECK(!gate.evaluate(AxisId::Altitude, 100.0F, kGen).send);

    // Поколение B: та же уставка обязана уйти ОБЕИМ осям.
    const auto azimuth = gate.evaluate(AxisId::Azimuth, 100.0F, kGen + 1);
    CHECK(azimuth.send);
    const auto altitude = gate.evaluate(AxisId::Altitude, 100.0F, kGen + 1);
    CHECK(altitude.send);
}

void generationResetsBothAxesReverseOrder()
{
    control::RateGate gate;
    gate.configure({8.0F});
    for (const auto id : {AxisId::Altitude, AxisId::Azimuth}) {
        const auto decision = gate.evaluate(id, 100.0F, kGen);
        gate.noteEnqueued(id, decision.value, kGen);
    }

    const auto altitude = gate.evaluate(AxisId::Altitude, 100.0F, kGen + 1);
    CHECK(altitude.send);
    const auto azimuth = gate.evaluate(AxisId::Azimuth, 100.0F, kGen + 1);
    CHECK(azimuth.send);
}

void noteEnqueuedWithoutEvaluate()
{
    control::RateGate gate;
    gate.configure({8.0F});
    gate.noteEnqueued(AxisId::Azimuth, 100.0F, kGen);
    gate.noteEnqueued(AxisId::Altitude, 100.0F, kGen);

    // Первым в новом поколении идёт noteEnqueued: он обязан инвалидировать
    // слот второй оси, а не оставить в нём прежнюю уставку.
    gate.noteEnqueued(AxisId::Azimuth, 100.0F, kGen + 1);
    const auto altitude = gate.evaluate(AxisId::Altitude, 100.0F, kGen + 1);
    CHECK(altitude.send);
}

void generationWithPartialFailure()
{
    control::RateGate gate;
    gate.configure({8.0F});
    const auto azimuth = gate.evaluate(AxisId::Azimuth, 100.0F, kGen);
    gate.noteEnqueued(AxisId::Azimuth, azimuth.value, kGen);
    const auto altitude = gate.evaluate(AxisId::Altitude, 100.0F, kGen);
    CHECK(altitude.send);
    gate.noteFailed(AxisId::Altitude);

    const auto a2 = gate.evaluate(AxisId::Azimuth, 100.0F, kGen + 1);
    CHECK(a2.send);
    const auto b2 = gate.evaluate(AxisId::Altitude, 100.0F, kGen + 1);
    CHECK(b2.send);
}

void axisIndependence()
{
    control::RateGate gate;
    gate.configure({8.0F});
    const auto azimuth = gate.evaluate(AxisId::Azimuth, 100.0F, kGen);
    gate.noteEnqueued(AxisId::Azimuth, azimuth.value, kGen);

    const auto altitude = gate.evaluate(AxisId::Altitude, 100.0F, kGen);
    CHECK(altitude.send);
    gate.noteFailed(AxisId::Altitude);

    // Неудача второй оси не заставляет переотправлять первую.
    CHECK(!gate.evaluate(AxisId::Azimuth, 100.0F, kGen).send);
    CHECK(gate.evaluate(AxisId::Altitude, 100.0F, kGen).send);
    CHECK_NEAR(gate.lastEnqueued(AxisId::Azimuth), 100.0, 1e-3);
}

void helpers()
{
    control::RateGate gate;
    gate.configure({8.0F});
    CHECK(!gate.hasEnqueuedNonZero(AxisId::Azimuth));
    const auto value = gate.evaluate(AxisId::Azimuth, 80.0F, kGen);
    gate.noteEnqueued(AxisId::Azimuth, value.value, kGen);
    CHECK(gate.hasEnqueuedNonZero(AxisId::Azimuth));
    gate.reset();
    CHECK(!gate.hasEnqueuedNonZero(AxisId::Azimuth));
}

}  // namespace

int main()
{
    firstValueIsSent();
    duplicateIsSuppressed();
    zeroWinsOverPending();
    pendingRepeatsCurrentValue();
    signChangeIsSent();
    generationResetsBothAxes();
    generationResetsBothAxesReverseOrder();
    noteEnqueuedWithoutEvaluate();
    generationWithPartialFailure();
    axisIndependence();
    helpers();
    return TEST_RESULT();
}
