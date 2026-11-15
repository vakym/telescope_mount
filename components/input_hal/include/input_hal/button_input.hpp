/**
 * @file button_input.hpp
 * @brief Опрос кнопки джойстика, активный низкий уровень.
 */

#pragma once

#include "board/board.hpp"
#include "driver/gpio.h"
#include "esp_err.h"

namespace input_hal {

class ButtonInput {
public:
    esp_err_t init(const board::JoystickPins &pins);

    /// Активный низкий уровень: нажатие — это уровень 0.
    bool isPressed() const noexcept;

    void deinit();

private:
    gpio_num_t pin_ = GPIO_NUM_NC;
    bool initialized_ = false;
};

}  // namespace input_hal
