/**
 * @file test_button_debouncer.cpp
 * @brief Проверки антидребезга и событий кнопки.
 */

#include <cstdint>

#include "check.hpp"
#include "input/button_debouncer.hpp"

namespace {

using input::ButtonEvent;

void bounceIgnored()
{
    input::ButtonDebouncer button;
    button.configure({40, 1000});
    button.reset(false, 0);

    CHECK(button.update(true, 10).count == 0);
    CHECK(button.update(false, 20).count == 0);
    CHECK(button.update(true, 30).count == 0);
    CHECK(button.update(false, 40).count == 0);
    CHECK(!button.pressed());
}

void pressEvent()
{
    input::ButtonDebouncer button;
    button.configure({40, 1000});
    button.reset(false, 0);

    CHECK(button.update(true, 10).count == 0);
    const auto events = button.update(true, 50);
    CHECK(events.count == 1);
    CHECK(events.values[0] == ButtonEvent::Pressed);
    CHECK(button.pressed());
}

void shortClick()
{
    input::ButtonDebouncer button;
    button.configure({40, 1000});
    button.reset(false, 0);
    (void)button.update(true, 10);
    (void)button.update(true, 50);

    CHECK(button.update(false, 100).count == 0);
    const auto events = button.update(false, 140);
    CHECK(events.count == 2);
    CHECK(events.values[0] == ButtonEvent::Released);
    CHECK(events.values[1] == ButtonEvent::Click);
}

void longPress()
{
    input::ButtonDebouncer button;
    button.configure({40, 1000});
    button.reset(false, 0);
    (void)button.update(true, 10);
    (void)button.update(true, 50);

    CHECK(button.update(true, 1049).count == 0);
    const auto long_event = button.update(true, 1050);
    CHECK(long_event.count == 1);
    CHECK(long_event.values[0] == ButtonEvent::LongPress);

    // Повторного LongPress нет.
    CHECK(button.update(true, 1100).count == 0);

    CHECK(button.update(false, 1200).count == 0);
    const auto release = button.update(false, 1240);
    CHECK(release.count == 1);
    CHECK(release.values[0] == ButtonEvent::Released);
}

void resetWhilePressed()
{
    input::ButtonDebouncer button;
    button.configure({40, 1000});
    button.reset(true, 0);

    // Удержание существующего нажатия не даёт LongPress.
    CHECK(button.update(true, 2000).count == 0);
    CHECK(button.update(false, 2010).count == 0);
    const auto release = button.update(false, 2050);
    CHECK(release.count == 1);
    CHECK(release.values[0] == ButtonEvent::Released);

    // Следующее нажатие обрабатывается обычно.
    CHECK(button.update(true, 2100).count == 0);
    const auto press = button.update(true, 2140);
    CHECK(press.count == 1);
    CHECK(press.values[0] == ButtonEvent::Pressed);
}

void releaseAtLongBoundary()
{
    input::ButtonDebouncer button;
    button.configure({40, 1000});
    button.reset(false, 0);
    (void)button.update(true, 10);
    (void)button.update(true, 50);  // подтверждённый Pressed, старт удержания

    CHECK(button.update(false, 1050).count == 0);  // начало дребезга отпускания
    const auto events = button.update(false, 1090);
    CHECK(events.count == 2);
    CHECK(events.values[0] == ButtonEvent::LongPress);
    CHECK(events.values[1] == ButtonEvent::Released);
}

void timeWraparound()
{
    input::ButtonDebouncer button;
    button.configure({40, 1000});
    button.reset(false, 0xFFFFFFF0U);

    CHECK(button.update(true, 0xFFFFFFF0U).count == 0);
    CHECK(button.update(true, 0x10U).count == 0);  // 32 мс < 40
    const auto press = button.update(true, 0x20U); // 48 мс >= 40
    CHECK(press.count == 1);
    CHECK(press.values[0] == ButtonEvent::Pressed);

    CHECK(button.update(false, 0x30U).count == 0);
    const auto release = button.update(false, 0x60U);
    CHECK(release.count == 2);
    CHECK(release.values[0] == ButtonEvent::Released);
    CHECK(release.values[1] == ButtonEvent::Click);
}

void invalidConfig()
{
    input::ButtonDebouncer button;
    CHECK(!button.isValid({0, 1000}));
    CHECK(!button.isValid({40, 0}));
    CHECK(button.isValid({1, 1}));

    // Невалидные значения заменяются безопасными по умолчанию.
    button.configure({0, 0});
    button.reset(false, 0);
    (void)button.update(true, 10);
    const auto press = button.update(true, 50);  // 40 мс по умолчанию
    CHECK(press.count == 1);
    CHECK(press.values[0] == ButtonEvent::Pressed);
}

}  // namespace

int main()
{
    bounceIgnored();
    pressEvent();
    shortClick();
    longPress();
    resetWhilePressed();
    releaseAtLongBoundary();
    timeWraparound();
    invalidConfig();
    return TEST_RESULT();
}
