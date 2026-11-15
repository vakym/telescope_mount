/**
 * @file button_debouncer.hpp
 * @brief Антидребезг кнопки, одиночный и двойной клик.
 *
 * Окно двойного нажатия включательное: второе подтверждённое нажатие
 * засчитывается при `elapsed <= double_click_ms`. Одиночный `Click`
 * откладывается до истечения окна, поэтому потребитель никогда не получает
 * `Click` и `DoubleClick` за один жест. При `double_click_ms == 0` режим
 * выключен и `Click` выдаётся сразу вместе с `Released`.
 */

#pragma once

#include <cstdint>

namespace input {

struct ButtonConfig {
    std::uint32_t debounce_ms = 40;
    std::uint32_t long_press_ms = 1000;
    /// Окно двойного нажатия, 0 — режим выключен.
    std::uint32_t double_click_ms = 350;
};

enum class ButtonEvent : std::uint8_t {
    None,
    Pressed,
    Released,
    Click,
    LongPress,
    /// Добавлено после LongPress: числовые значения прежних событий
    /// не сдвигаются.
    DoubleClick,
};

/// До двух событий за один вызов, без динамической памяти.
struct ButtonEvents {
    ButtonEvent values[2]{};
    std::uint8_t count = 0;
};

class ButtonListener {
public:
    virtual ~ButtonListener() = default;

    /**
     * @brief Вызывается из задачи опроса.
     *
     * Обязан быть неблокирующим и не вызывать start()/stop() сервиса.
     */
    virtual void onButtonEvent(ButtonEvent event) = 0;
};

class ButtonDebouncer {
public:
    /**
     * true, если debounce_ms > 0, long_press_ms > 0, а double_click_ms
     * равно нулю или не меньше debounce_ms: окно короче антидребезга не
     * позволяет распознать второе нажатие.
     */
    bool isValid(const ButtonConfig &config) noexcept;

    /// false = конфигурация отвергнута; значения по умолчанию не подставляются.
    bool configure(const ButtonConfig &config) noexcept;

    /// Фиксирует исходное состояние без генерации события.
    void reset(bool raw_pressed, std::uint32_t now_ms) noexcept;

    /// Возвращает 0, 1 или 2 события за вызов.
    ButtonEvents update(bool raw_pressed, std::uint32_t now_ms) noexcept;

    /// Подтверждённое антидребезгом состояние.
    bool pressed() const noexcept { return stable_pressed_; }

private:
    ButtonConfig config_{};
    bool initialized_ = false;
    bool stable_pressed_ = false;
    bool raw_pressed_ = false;
    bool press_trusted_ = false;
    bool long_sent_ = false;
    /// Одиночный клик отложен до истечения окна double_click_ms.
    bool pending_click_ = false;
    /// Текущее нажатие — второе в окне, кандидат на DoubleClick.
    bool second_press_ = false;
    std::uint32_t raw_since_ms_ = 0;
    std::uint32_t press_started_ms_ = 0;
    std::uint32_t pending_since_ms_ = 0;
};

}  // namespace input
