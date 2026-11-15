/**
 * @file button_input.cpp
 * @brief GPIO кнопки: вход с подтяжкой, активный низкий уровень.
 */

#include "input_hal/button_input.hpp"

#include <cstdint>

namespace input_hal {

esp_err_t ButtonInput::init(const board::JoystickPins &pins)
{
    if (initialized_) {
        return ESP_OK;
    }

    pin_ = pins.sw;

    gpio_config_t config{};
    config.pin_bit_mask = 1ULL << static_cast<std::uint32_t>(pin_);
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_ENABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;

    const esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) {
        pin_ = GPIO_NUM_NC;
        return err;
    }

    initialized_ = true;
    return ESP_OK;
}

bool ButtonInput::isPressed() const noexcept
{
    if (!initialized_ || pin_ == GPIO_NUM_NC) {
        return false;
    }
    return gpio_get_level(pin_) == 0;
}

void ButtonInput::deinit()
{
    if (initialized_ && pin_ != GPIO_NUM_NC) {
        (void)gpio_reset_pin(pin_);
    }
    pin_ = GPIO_NUM_NC;
    initialized_ = false;
}

}  // namespace input_hal
