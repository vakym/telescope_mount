/**
 * @file board.cpp
 * @brief Инициализация линий ULN2003 обеих осей.
 */

#include "board/board.hpp"

#include <cstdint>

#include "esp_check.h"

namespace board {
namespace {

constexpr const char *kTag = "board";

/// Маска всех восьми линий обоих драйверов.
constexpr std::uint64_t coilMask()
{
    std::uint64_t mask = 0;
    for (gpio_num_t pin : kMotorPins.azimuth.in) {
        mask |= 1ULL << static_cast<std::uint32_t>(pin);
    }
    for (gpio_num_t pin : kMotorPins.altitude.in) {
        mask |= 1ULL << static_cast<std::uint32_t>(pin);
    }
    return mask;
}

}  // namespace

esp_err_t init()
{
    gpio_config_t cfg{};
    cfg.pin_bit_mask = coilMask();
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;

    ESP_RETURN_ON_ERROR(gpio_config(&cfg), kTag, "gpio_config failed");

    // Явно низкий уровень на всех линиях: катушки выключены до первой команды.
    for (gpio_num_t pin : kMotorPins.azimuth.in) {
        ESP_RETURN_ON_ERROR(gpio_set_level(pin, 0), kTag, "gpio_set_level failed");
    }
    for (gpio_num_t pin : kMotorPins.altitude.in) {
        ESP_RETURN_ON_ERROR(gpio_set_level(pin, 0), kTag, "gpio_set_level failed");
    }

    return ESP_OK;
}

}  // namespace board
