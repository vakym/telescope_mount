/**
 * @file button_debouncer.hpp
 * @brief Антидребезг кнопки и формирование событий.
 */

#pragma once

#include <cstdint>

namespace input {

struct ButtonConfig {
    std::uint32_t debounce_ms = 40;
    std::uint32_t long_press_ms = 1000;
};

enum class ButtonEvent : std::uint8_t {
    None,
    Pressed,
    Released,
    Click,
    LongPress,
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
    /// true, если debounce_ms > 0 и long_press_ms > 0.
    bool isValid(const ButtonConfig &config) noexcept;

    /// Невалидная конфигурация заменяется значениями по умолчанию.
    void configure(const ButtonConfig &config) noexcept;

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
    std::uint32_t raw_since_ms_ = 0;
    std::uint32_t press_started_ms_ = 0;
};

}  // namespace input
