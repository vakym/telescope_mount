/**
 * @file button_debouncer.cpp
 * @brief Антидребезг кнопки с активным низким уровнем на стороне ввода,
 *        отложенный одиночный клик и определение двойного нажатия.
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
    if (config.debounce_ms == 0 || config.long_press_ms == 0) {
        return false;
    }
    // Окно короче антидребезга не позволяет распознать второе нажатие.
    return config.double_click_ms == 0 ||
           config.double_click_ms >= config.debounce_ms;
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
    pending_click_ = false;
    second_press_ = false;
    raw_since_ms_ = 0;
    press_started_ms_ = 0;
    pending_since_ms_ = 0;
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
    pending_click_ = false;
    second_press_ = false;
    pending_since_ms_ = now_ms;
}

ButtonEvents ButtonDebouncer::update(bool raw_pressed, std::uint32_t now_ms) noexcept
{
    ButtonEvents events;
    if (!initialized_) {
        reset(raw_pressed, now_ms);
        return events;
    }

    // 1. Истечение окна отложенного одиночного клика. Проверка выполняется
    //    первой, поэтому граница окна детерминирована.
    if (pending_click_ &&
        elapsed(now_ms, pending_since_ms_) > config_.double_click_ms) {
        push(events, ButtonEvent::Click);
        pending_click_ = false;
    }

    // 2. Обнаружение изменения сырого уровня и антидребезг.
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
            // Длинное второе нажатие отменяет жест двойного нажатия.
            second_press_ = false;
        }
        return events;
    }

    if (elapsed(now_ms, raw_since_ms_) < config_.debounce_ms) {
        return events;
    }

    stable_pressed_ = raw_pressed;

    if (stable_pressed_) {
        // 3. Подтверждённое нажатие. Нажатие в окне отложенного клика
        //    снимает ожидание и делает жест кандидатом на DoubleClick.
        if (pending_click_) {
            pending_click_ = false;
            second_press_ = true;
        } else {
            second_press_ = false;
        }
        push(events, ButtonEvent::Pressed);
        press_trusted_ = true;
        press_started_ms_ = now_ms;
        long_sent_ = false;
        return events;
    }

    // 4. Подтверждённое отпускание.
    //    Порог long_press_ms, достигнутый к моменту отпускания, выдаёт
    //    LongPress перед Released: порядок [LongPress, Released].
    const bool long_reached = press_trusted_ && !long_sent_ &&
                              elapsed(now_ms, press_started_ms_) >=
                                  config_.long_press_ms;
    if (long_reached) {
        push(events, ButtonEvent::LongPress);
        long_sent_ = true;
    }

    push(events, ButtonEvent::Released);

    const bool suppress_click = long_sent_ || !press_trusted_;
    const bool was_second = second_press_;
    press_trusted_ = false;
    long_sent_ = false;
    second_press_ = false;

    if (suppress_click) {
        // Удержание или нажатие, существовавшее при reset(): клика нет.
        return events;
    }

    if (was_second) {
        push(events, ButtonEvent::DoubleClick);
    } else if (config_.double_click_ms == 0) {
        push(events, ButtonEvent::Click);
    } else {
        pending_click_ = true;
        pending_since_ms_ = now_ms;
    }
    return events;
}

}  // namespace input
