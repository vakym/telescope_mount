/**
 * @file coil_driver.cpp
 * @brief Запись фазовых состояний в линии ULN2003.
 */

#include "motor_hal/coil_driver.hpp"

#include "esp_check.h"
#include "motor_hal/coil_pattern.hpp"

namespace motor_hal {
namespace {

constexpr const char *kTag = "coil_driver";

}  // namespace

esp_err_t CoilDriver::init(const board::CoilPins &pins, motion::StepPattern pattern, bool invert)
{
    pins_ = pins;
    pattern_ = pattern;
    invert_ = invert;
    index_ = 0;
    energized_ = false;

    // Все линии — выход, низкий уровень: катушки выключены до первой команды.
    for (gpio_num_t pin : pins_.in) {
        ESP_RETURN_ON_ERROR(gpio_set_level(pin, 0), kTag, "gpio_set_level failed");
    }

    return ESP_OK;
}

void CoilDriver::applyPhase()
{
    const PhaseMask mask = tableFor(pattern_)[index_];
    for (std::size_t i = 0; i < 4; ++i) {
        std::uint32_t level = ((mask >> i) & 0x1U) != 0U ? 1U : 0U;
        if (invert_) {
            level ^= 1U;
        }
        (void)gpio_set_level(pins_.in[i], level);
    }
}

void CoilDriver::advance(motion::Direction direction)
{
    const std::size_t length = patternLength(pattern_);
    if (direction == motion::Direction::Forward) {
        index_ = static_cast<std::uint8_t>((index_ + 1U) % length);
    } else {
        index_ = static_cast<std::uint8_t>((index_ + length - 1U) % length);
    }
    energized_ = true;
    applyPhase();
}

void CoilDriver::release()
{
    for (gpio_num_t pin : pins_.in) {
        (void)gpio_set_level(pin, 0);
    }
    energized_ = false;
}

void CoilDriver::energize()
{
    if (!energized_) {
        applyPhase();
        energized_ = true;
    }
}

}  // namespace motor_hal
