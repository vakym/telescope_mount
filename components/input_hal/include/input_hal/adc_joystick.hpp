/**
 * @file adc_joystick.hpp
 * @brief Чтение VRx/VRy через adc_oneshot на ADC1.
 */

#pragma once

#include <cstdint>

#include "board/board.hpp"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "input/joystick_config.hpp"

namespace input_hal {

class AdcJoystick {
public:
    esp_err_t init(const board::JoystickPins &pins);

    /// Читает оба канала, усредняя CONFIG_JOY_ADC_OVERSAMPLE отсчётов.
    input::RawSample read() noexcept;

    std::uint32_t errorCount() const noexcept { return errors_; }

    /// Освобождает unit. Не вызывать, пока задача может читать ADC.
    void deinit();

private:
    adc_oneshot_unit_handle_t unit_ = nullptr;
    adc_channel_t channels_[input::kAxisCount] = {ADC_CHANNEL_6, ADC_CHANNEL_7};
    std::uint32_t errors_ = 0;
    bool initialized_ = false;
};

}  // namespace input_hal
