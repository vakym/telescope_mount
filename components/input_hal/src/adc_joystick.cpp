/**
 * @file adc_joystick.cpp
 * @brief adc_oneshot на ADC1: два канала с усреднением.
 */

#include "input_hal/adc_joystick.hpp"

#include "sdkconfig.h"

namespace input_hal {
namespace {

constexpr int kOversample =
    CONFIG_JOY_ADC_OVERSAMPLE > 0 ? CONFIG_JOY_ADC_OVERSAMPLE : 1;

}  // namespace

esp_err_t AdcJoystick::init(const board::JoystickPins &pins)
{
    if (initialized_) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t init_config{};
    init_config.unit_id = pins.adc_unit;
    init_config.ulp_mode = ADC_ULP_MODE_DISABLE;

    esp_err_t err = adc_oneshot_new_unit(&init_config, &unit_);
    if (err != ESP_OK) {
        unit_ = nullptr;
        return err;
    }

    channels_[0] = pins.vrx_channel;
    channels_[1] = pins.vry_channel;

    adc_oneshot_chan_cfg_t channel_config{};
    channel_config.atten = ADC_ATTEN_DB_12;
    channel_config.bitwidth = ADC_BITWIDTH_12;

    for (std::size_t i = 0; i < input::kAxisCount; ++i) {
        err = adc_oneshot_config_channel(unit_, channels_[i], &channel_config);
        if (err != ESP_OK) {
            deinit();
            return err;
        }
    }

    initialized_ = true;
    return ESP_OK;
}

input::RawSample AdcJoystick::read() noexcept
{
    input::RawSample result;
    if (!initialized_ || unit_ == nullptr) {
        return result;
    }

    int sum[input::kAxisCount] = {0, 0};
    for (int sample = 0; sample < kOversample; ++sample) {
        for (std::size_t axis = 0; axis < input::kAxisCount; ++axis) {
            int raw = 0;
            const esp_err_t err = adc_oneshot_read(unit_, channels_[axis], &raw);
            if (err != ESP_OK) {
                ++errors_;
                return input::RawSample{};
            }
            sum[axis] += raw;
        }
    }

    for (std::size_t axis = 0; axis < input::kAxisCount; ++axis) {
        const int average = (sum[axis] + kOversample / 2) / kOversample;
        result.counts[axis] = static_cast<std::uint16_t>(average);
    }
    result.valid = true;
    return result;
}

void AdcJoystick::deinit()
{
    if (unit_ != nullptr) {
        (void)adc_oneshot_del_unit(unit_);
        unit_ = nullptr;
    }
    initialized_ = false;
}

}  // namespace input_hal
