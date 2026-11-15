/**
 * @file button_debouncer.cpp
 * @brief Антидребезг кнопки с активным низким уровнем на стороне ввода.
 */

#include "input/button_debouncer.hpp"

#include <cstddef>

namespace input {
namespace {

constexpr std::uint8_t kMaxEvents = 2;

void push(ButtonEvents &events, ButtonEvent event) noexcept
{
    if (events.count < kMaxEvents) {
        events.values[events.count] = event;
        ++events.count;
    }
}

/// Беззнаковая разность устойчива к переполнению uint32_t.
std::uint32_t elapsed(std::uint32_t now_ms, std::uint32_t since_ms) noexcept
{
    return now_ms - since_ms;
}

}  // namespace

bool ButtonDebouncer::isValid(const ButtonConfig &config) noexcept
{
    return config.debounce_ms > 0 && config.long_press_ms > 0;
}

bool ButtonDebouncer::configure(const ButtonConfig &config) noexcept
{
    if (!isValid(config)) {
        return false;
    }
    config_ = config;
    initialized_ = false;
    stable_pressed_ = false;
    raw_pressed_ = false;
    press_trusted_ = false;
    long_sent_ = false;
    raw_since_ms_ = 0;
    press_started_ms_ = 0;
    return true;
}

void ButtonDebouncer::reset(bool raw_pressed, std::uint32_t now_ms) noexcept
{
    initialized_ = true;
    stable_pressed_ = raw_pressed;
    raw_pressed_ = raw_pressed;
    raw_since_ms_ = now_ms;
    press_started_ms_ = now_ms;
    // Нажатие, существовавшее при запуске, не даёт Click и LongPress.
    press_trusted_ = false;
    long_sent_ = false;
}

ButtonEvents ButtonDebouncer::update(bool raw_pressed, std::uint32_t now_ms) noexcept
{
    ButtonEvents events;
    if (!initialized_) {
        reset(raw_pressed, now_ms);
        return events;
    }

    if (raw_pressed != raw_pressed_) {
        raw_pressed_ = raw_pressed;
        raw_since_ms_ = now_ms;
        return events;
    }

    if (raw_pressed == stable_pressed_) {
        if (stable_pressed_ && press_trusted_ && !long_sent_ &&
            elapsed(now_ms, press_started_ms_) >= config_.long_press_ms) {
            push(events, ButtonEvent::LongPress);
            long_sent_ = true;
        }
        return events;
    }

    if (elapsed(now_ms, raw_since_ms_) < config_.debounce_ms) {
        return events;
    }

    stable_pressed_ = raw_pressed;
    if (stable_pressed_) {
        push(events, ButtonEvent::Pressed);
        press_trusted_ = true;
        press_started_ms_ = now_ms;
        long_sent_ = false;
        return events;
    }

    // Отпускание на границе: LongPress ещё не был выдан, но порог уже
    // достигнут. Он идёт перед Released, чтобы порядок событий был
    // [LongPress, Released].
    if (press_trusted_ && !long_sent_ &&
        elapsed(now_ms, press_started_ms_) >= config_.long_press_ms) {
        push(events, ButtonEvent::LongPress);
        long_sent_ = true;
    }
    push(events, ButtonEvent::Released);
    if (press_trusted_ && !long_sent_) {
        push(events, ButtonEvent::Click);
    }
    press_trusted_ = false;
    long_sent_ = false;
    return events;
}

}  // namespace input
