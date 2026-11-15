/**
 * @file test_button_debouncer.cpp
 * @brief Проверки антидребезга, одиночного и двойного нажатия.
 *
 * Режим совместимости (`double_click_ms = 0`) проверяется явной передачей
 * третьего поля: `ButtonConfig{40, 1000}` теперь означает окно 350 мс и
 * другое поведение.
 */

#include <cstdint>

#include "check.hpp"
#include "input/button_debouncer.hpp"

namespace {

using input::ButtonConfig;
using input::ButtonDebouncer;
using input::ButtonEvent;

constexpr ButtonConfig kLegacy{40, 1000, 0};
constexpr ButtonConfig kDouble{40, 1000, 350};

void bounceIgnored()
{
    ButtonDebouncer button;
    button.configure(kLegacy);
    button.reset(false, 0);

    CHECK(button.update(true, 10).count == 0);
    CHECK(button.update(false, 20).count == 0);
    CHECK(button.update(true, 30).count == 0);
    CHECK(button.update(false, 40).count == 0);
    CHECK(!button.pressed());
}

void pressEvent()
{
    ButtonDebouncer button;
    button.configure(kLegacy);
    button.reset(false, 0);

    CHECK(button.update(true, 10).count == 0);
    const auto events = button.update(true, 50);
    CHECK(events.count == 1);
    CHECK(events.values[0] == ButtonEvent::Pressed);
    CHECK(button.pressed());
}

void shortClickImmediate()
{
    ButtonDebouncer button;
    button.configure(kLegacy);
    button.reset(false, 0);
    (void)button.update(true, 10);
    (void)button.update(true, 50);

    CHECK(button.update(false, 100).count == 0);
    const auto events = button.update(false, 140);
    CHECK(events.count == 2);
    CHECK(events.values[0] == ButtonEvent::Released);
    CHECK(events.values[1] == ButtonEvent::Click);
}

void shortClickDeferred()
{
    ButtonDebouncer button;
    button.configure(kDouble);
    button.reset(false, 0);
    CHECK(button.update(true, 10).count == 0);
    CHECK(button.update(true, 50).count == 1);

    CHECK(button.update(false, 100).count == 0);
    const auto release = button.update(false, 140);
    CHECK(release.count == 1);
    CHECK(release.values[0] == ButtonEvent::Released);

    // Окно ещё не истекло: 350 мс включительно.
    CHECK(button.update(false, 200).count == 0);
    CHECK(button.update(false, 490).count == 0);

    const auto click = button.update(false, 491);
    CHECK(click.count == 1);
    CHECK(click.values[0] == ButtonEvent::Click);
}

void doubleClickInWindow()
{
    ButtonDebouncer button;
    button.configure(kDouble);
    button.reset(false, 0);
    (void)button.update(true, 10);
    (void)button.update(true, 50);
    (void)button.update(false, 100);
    const auto first = button.update(false, 140);
    CHECK(first.count == 1);
    CHECK(first.values[0] == ButtonEvent::Released);

    // Второе нажатие ровно на границе окна: 490 - 140 = 350.
    CHECK(button.update(true, 450).count == 0);
    const auto press = button.update(true, 490);
    CHECK(press.count == 1);
    CHECK(press.values[0] == ButtonEvent::Pressed);

    CHECK(button.update(false, 550).count == 0);
    const auto second = button.update(false, 590);
    CHECK(second.count == 2);
    CHECK(second.values[0] == ButtonEvent::Released);
    CHECK(second.values[1] == ButtonEvent::DoubleClick);

    // Отложенного Click за первым кликом нет.
    CHECK(button.update(false, 700).count == 0);
    CHECK(button.update(false, 1000).count == 0);
}

void twoClicksApart()
{
    ButtonDebouncer button;
    button.configure(kDouble);
    button.reset(false, 0);

    (void)button.update(true, 10);
    (void)button.update(true, 50);
    (void)button.update(false, 100);
    const auto first_release = button.update(false, 140);
    CHECK(first_release.count == 1);
    CHECK(first_release.values[0] == ButtonEvent::Released);

    const auto first_click = button.update(false, 491);
    CHECK(first_click.count == 1);
    CHECK(first_click.values[0] == ButtonEvent::Click);

    (void)button.update(true, 600);
    const auto press = button.update(true, 640);
    CHECK(press.count == 1);
    CHECK(press.values[0] == ButtonEvent::Pressed);

    (void)button.update(false, 700);
    const auto second_release = button.update(false, 740);
    CHECK(second_release.count == 1);
    CHECK(second_release.values[0] == ButtonEvent::Released);

    const auto second_click = button.update(false, 1091);
    CHECK(second_click.count == 1);
    CHECK(second_click.values[0] == ButtonEvent::Click);
}

void secondPressLate()
{
    ButtonDebouncer button;
    button.configure(kDouble);
    button.reset(false, 0);

    (void)button.update(true, 10);
    (void)button.update(true, 50);
    (void)button.update(false, 100);
    CHECK(button.update(false, 140).count == 1);

    // Нажатие через 351 мс: ожидание истекает в том же вызове.
    CHECK(button.update(true, 450).count == 0);
    const auto press = button.update(true, 491);
    CHECK(press.count == 2);
    CHECK(press.values[0] == ButtonEvent::Click);
    CHECK(press.values[1] == ButtonEvent::Pressed);

    (void)button.update(false, 550);
    const auto release = button.update(false, 590);
    CHECK(release.count == 1);
    CHECK(release.values[0] == ButtonEvent::Released);
}

void secondPressLong()
{
    ButtonDebouncer button;
    button.configure(kDouble);
    button.reset(false, 0);

    (void)button.update(true, 10);
    (void)button.update(true, 50);
    (void)button.update(false, 100);
    (void)button.update(false, 140);

    (void)button.update(true, 200);
    const auto press = button.update(true, 240);
    CHECK(press.count == 1);
    CHECK(press.values[0] == ButtonEvent::Pressed);

    // Порог отсчитывается от подтверждённого нажатия 240: 240 + 1000.
    CHECK(button.update(true, 1239).count == 0);
    const auto long_event = button.update(true, 1240);
    CHECK(long_event.count == 1);
    CHECK(long_event.values[0] == ButtonEvent::LongPress);

    (void)button.update(false, 1300);
    const auto release = button.update(false, 1340);
    CHECK(release.count == 1);
    CHECK(release.values[0] == ButtonEvent::Released);

    // Ни Click, ни DoubleClick; ожидания больше нет.
    CHECK(button.update(false, 1800).count == 0);
}

void firstPressLong()
{
    ButtonDebouncer button;
    button.configure(kDouble);
    button.reset(false, 0);

    (void)button.update(true, 10);
    (void)button.update(true, 50);
    CHECK(button.update(true, 1049).count == 0);
    const auto long_event = button.update(true, 1050);
    CHECK(long_event.count == 1);
    CHECK(long_event.values[0] == ButtonEvent::LongPress);

    (void)button.update(false, 1200);
    const auto release = button.update(false, 1240);
    CHECK(release.count == 1);
    CHECK(release.values[0] == ButtonEvent::Released);

    // Отложенного клика нет.
    CHECK(button.update(false, 1700).count == 0);
}

void thirdPressAfterDouble()
{
    ButtonDebouncer button;
    button.configure(kDouble);
    button.reset(false, 0);

    (void)button.update(true, 10);
    (void)button.update(true, 50);
    (void)button.update(false, 100);
    (void)button.update(false, 140);
    (void)button.update(true, 200);
    (void)button.update(true, 240);
    (void)button.update(false, 300);
    const auto double_click = button.update(false, 340);
    CHECK(double_click.count == 2);
    CHECK(double_click.values[1] == ButtonEvent::DoubleClick);

    // Новый жест: третье нажатие снова откладывает клик.
    (void)button.update(true, 400);
    const auto press = button.update(true, 440);
    CHECK(press.count == 1);
    CHECK(press.values[0] == ButtonEvent::Pressed);
    (void)button.update(false, 500);
    const auto release = button.update(false, 540);
    CHECK(release.count == 1);
    CHECK(release.values[0] == ButtonEvent::Released);

    const auto click = button.update(false, 891);
    CHECK(click.count == 1);
    CHECK(click.values[0] == ButtonEvent::Click);
}

void resetAndConfigureClearPending()
{
    ButtonDebouncer button;
    button.configure(kDouble);
    button.reset(false, 0);
    (void)button.update(true, 10);
    (void)button.update(true, 50);
    (void)button.update(false, 100);
    CHECK(button.update(false, 140).count == 1);

    button.reset(false, 200);
    CHECK(button.update(false, 900).count == 0);

    // Повтор после configure().
    (void)button.update(true, 1000);
    (void)button.update(true, 1040);
    (void)button.update(false, 1100);
    CHECK(button.update(false, 1140).count == 1);
    CHECK(button.configure(kDouble));
    CHECK(button.update(false, 1600).count == 0);
}

void resetWhilePressed()
{
    ButtonDebouncer button;
    button.configure(kLegacy);
    button.reset(true, 0);

    CHECK(button.update(true, 2000).count == 0);
    CHECK(button.update(false, 2010).count == 0);
    const auto release = button.update(false, 2050);
    CHECK(release.count == 1);
    CHECK(release.values[0] == ButtonEvent::Released);

    CHECK(button.update(true, 2100).count == 0);
    const auto press = button.update(true, 2140);
    CHECK(press.count == 1);
    CHECK(press.values[0] == ButtonEvent::Pressed);
}

void releaseAtLongBoundary()
{
    ButtonDebouncer button;
    button.configure(kLegacy);
    button.reset(false, 0);
    (void)button.update(true, 10);
    (void)button.update(true, 50);

    CHECK(button.update(false, 1050).count == 0);
    const auto events = button.update(false, 1090);
    CHECK(events.count == 2);
    CHECK(events.values[0] == ButtonEvent::LongPress);
    CHECK(events.values[1] == ButtonEvent::Released);
}

void timeWraparound()
{
    ButtonDebouncer button;
    button.configure(kLegacy);
    button.reset(false, 0xFFFFFFF0U);

    CHECK(button.update(true, 0xFFFFFFF0U).count == 0);
    CHECK(button.update(true, 0x10U).count == 0);
    const auto press = button.update(true, 0x20U);
    CHECK(press.count == 1);
    CHECK(press.values[0] == ButtonEvent::Pressed);

    CHECK(button.update(false, 0x30U).count == 0);
    const auto release = button.update(false, 0x60U);
    CHECK(release.count == 2);
    CHECK(release.values[0] == ButtonEvent::Released);
    CHECK(release.values[1] == ButtonEvent::Click);
}

void windowWraparound()
{
    ButtonDebouncer button;
    button.configure(kDouble);
    button.reset(false, 0xFFFFFF00U);

    // Нажатие 0x10 -> подтверждение 0x50, отпускание 0x60 -> 0xA0.
    CHECK(button.update(true, 0xFFFFFF10U).count == 0);
    CHECK(button.update(true, 0xFFFFFF50U).count == 1);
    CHECK(button.update(false, 0xFFFFFF60U).count == 0);
    const auto release = button.update(false, 0xFFFFFFA0U);
    CHECK(release.count == 1);
    CHECK(release.values[0] == ButtonEvent::Released);

    // 0xFE - 0xFFFFFFA0 = 350, окно ещё не истекло.
    CHECK(button.update(false, 0xFEU).count == 0);
    const auto click = button.update(false, 0xFFU);
    CHECK(click.count == 1);
    CHECK(click.values[0] == ButtonEvent::Click);
}

void invalidConfig()
{
    ButtonDebouncer button;
    CHECK(!button.isValid({0, 1000, 350}));
    CHECK(!button.isValid({40, 0, 350}));
    // Окно короче антидребезга отвергается.
    CHECK(!button.isValid({40, 1000, 1}));
    CHECK(!button.isValid({40, 1000, 39}));
    CHECK(button.isValid({40, 1000, 40}));
    CHECK(button.isValid({40, 1000, 0}));
    CHECK(button.isValid({1, 1, 0}));

    CHECK(!button.configure({0, 0, 0}));
    CHECK(!button.configure({0, 1000, 350}));
    CHECK(!button.configure({40, 0, 350}));
    CHECK(!button.configure({40, 1000, 10}));

    CHECK(button.configure(kDouble));
    button.reset(false, 0);
    (void)button.update(true, 10);
    const auto press = button.update(true, 50);
    CHECK(press.count == 1);
    CHECK(press.values[0] == ButtonEvent::Pressed);
}

}  // namespace

int main()
{
    bounceIgnored();
    pressEvent();
    shortClickImmediate();
    shortClickDeferred();
    doubleClickInWindow();
    twoClicksApart();
    secondPressLate();
    secondPressLong();
    firstPressLong();
    thirdPressAfterDouble();
    resetAndConfigureClearPending();
    resetWhilePressed();
    releaseAtLongBoundary();
    timeWraparound();
    windowWraparound();
    invalidConfig();
    return TEST_RESULT();
}
