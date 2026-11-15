/**
 * @file test_sample_gate.cpp
 * @brief Проверки последовательностей ошибок и успехов ADC.
 */

#include <cstdint>

#include "check.hpp"
#include "input/button_debouncer.hpp"
#include "input/sample_gate.hpp"

namespace {

using input::ButtonEvent;
using input::SampleGateAction;

void errorsThenSuccess()
{
    input::SampleGate gate;
    gate.configure({5, 5});

    for (int i = 0; i < 4; ++i) {
        CHECK(gate.update(false) == SampleGateAction::Ignore);
    }
    CHECK(!gate.faulted());
    CHECK(!gate.outputValid());
    CHECK(gate.consecutiveErrors() == 4);

    CHECK(gate.update(true) == SampleGateAction::Process);
    CHECK(!gate.faulted());
    CHECK(gate.outputValid());
    CHECK(gate.consecutiveErrors() == 0);
}

void faultAndInterruptedRecovery()
{
    input::SampleGate gate;
    gate.configure({5, 5});

    for (int i = 0; i < 5; ++i) {
        CHECK(gate.update(false) == SampleGateAction::Ignore);
    }
    CHECK(gate.faulted());
    CHECK(gate.consecutiveErrors() == 5);

    for (int i = 0; i < 4; ++i) {
        CHECK(gate.update(true) == SampleGateAction::Ignore);
    }
    CHECK(gate.faulted());
    CHECK(gate.consecutiveSuccesses() == 4);

    // Ошибка сбрасывает серию восстановления.
    CHECK(gate.update(false) == SampleGateAction::Ignore);
    CHECK(gate.faulted());
    CHECK(gate.consecutiveSuccesses() == 0);

    for (int i = 0; i < 4; ++i) {
        CHECK(gate.update(true) == SampleGateAction::Ignore);
    }
    CHECK(gate.faulted());

    CHECK(gate.update(true) == SampleGateAction::RecoverAndProcess);
    CHECK(!gate.faulted());
    CHECK(gate.outputValid());
}

void countersSaturate()
{
    input::SampleGate gate;
    gate.configure({5, 5});

    for (int i = 0; i < 20; ++i) {
        (void)gate.update(false);
    }
    CHECK(gate.consecutiveErrors() == 5);
    CHECK(gate.faulted());

    for (int i = 0; i < 5; ++i) {
        (void)gate.update(true);
    }
    CHECK(gate.consecutiveSuccesses() == 5);
    CHECK(!gate.faulted());

    (void)gate.update(true);
    CHECK(gate.consecutiveSuccesses() == 5);
}

void resetClears()
{
    input::SampleGate gate;
    gate.configure({2, 2});
    (void)gate.update(false);
    (void)gate.update(false);
    CHECK(gate.faulted());

    gate.reset();
    CHECK(!gate.faulted());
    CHECK(!gate.outputValid());
    CHECK(gate.consecutiveErrors() == 0);
    CHECK(gate.consecutiveSuccesses() == 0);
}

void invalidConfig()
{
    input::SampleGate gate;
    CHECK(!gate.isValid({0, 5}));
    CHECK(!gate.isValid({5, 0}));
    CHECK(gate.isValid({1, 1}));

    // Невалидная конфигурация заменяется безопасными значениями 5/5.
    gate.configure({0, 0});
    for (int i = 0; i < 5; ++i) {
        (void)gate.update(false);
    }
    CHECK(gate.faulted());
}

void buttonIndependentFromAdc()
{
    input::SampleGate gate;
    gate.configure({5, 5});

    input::ButtonDebouncer button;
    button.configure({40, 1000});
    button.reset(false, 0);

    ButtonEvent recorded[4]{};
    std::uint8_t recorded_count = 0;

    for (std::uint32_t now = 20; now <= 140; now += 20) {
        const bool raw_pressed = now >= 20 && now < 100;
        const auto events = button.update(raw_pressed, now);
        for (std::uint8_t i = 0; i < events.count && recorded_count < 4; ++i) {
            recorded[recorded_count++] = events.values[i];
        }
        // ADC всё время ошибается, кнопка обязана работать независимо.
        (void)gate.update(false);
    }

    CHECK(gate.faulted());
    CHECK(recorded_count == 3);
    CHECK(recorded[0] == ButtonEvent::Pressed);
    CHECK(recorded[1] == ButtonEvent::Released);
    CHECK(recorded[2] == ButtonEvent::Click);
}

}  // namespace

int main()
{
    errorsThenSuccess();
    faultAndInterruptedRecovery();
    countersSaturate();
    resetClears();
    invalidConfig();
    buttonIndependentFromAdc();
    return TEST_RESULT();
}
