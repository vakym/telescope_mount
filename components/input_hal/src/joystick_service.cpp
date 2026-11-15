/**
 * @file joystick_service.cpp
 * @brief Задача опроса, калибровка, свежесть данных и жизненный цикл.
 */

#include "input_hal/joystick_service.hpp"

#include <cstdint>

#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

namespace input_hal {
namespace {

constexpr const char *kTag = "joystick";

TickType_t samplePeriodTicks() noexcept
{
    const TickType_t ticks = pdMS_TO_TICKS(CONFIG_JOY_SAMPLE_PERIOD_MS);
    return ticks == 0 ? 1 : ticks;
}

TickType_t timeoutTicks(std::uint32_t ms) noexcept
{
    const TickType_t ticks = pdMS_TO_TICKS(ms);
    return ticks == 0 ? 1 : ticks;
}

input::JoystickConfig makeConfig() noexcept
{
    input::JoystickConfig config;
    for (std::size_t i = 0; i < input::kAxisCount; ++i) {
        config.axis[i].center = static_cast<float>(CONFIG_JOY_DEFAULT_CENTER);
        config.axis[i].min = 0.0F;
        config.axis[i].max = 4095.0F;
    }
#ifdef CONFIG_JOY_INVERT_X
    config.axis[0].invert = true;
#endif
#ifdef CONFIG_JOY_INVERT_Y
    config.axis[1].invert = true;
#endif
    config.dead_zone =
        static_cast<float>(CONFIG_JOY_DEAD_ZONE_PERMILLE) / 1000.0F;
    config.dead_zone_exit =
        static_cast<float>(CONFIG_JOY_DEAD_ZONE_EXIT_PERMILLE) / 1000.0F;
    config.filter_alpha =
        static_cast<float>(CONFIG_JOY_FILTER_ALPHA_X100) / 100.0F;
    return config;
}

input::SampleGateConfig makeGateConfig() noexcept
{
    return input::SampleGateConfig{
        static_cast<std::uint32_t>(CONFIG_JOY_ERROR_THRESHOLD),
        static_cast<std::uint32_t>(CONFIG_JOY_RECOVERY_THRESHOLD)};
}

input::ButtonConfig makeButtonConfig() noexcept
{
    return input::ButtonConfig{
        static_cast<std::uint32_t>(CONFIG_JOY_BUTTON_DEBOUNCE_MS),
        static_cast<std::uint32_t>(CONFIG_JOY_BUTTON_LONG_PRESS_MS),
        static_cast<std::uint32_t>(CONFIG_JOY_DOUBLE_CLICK_MS)};
}

}  // namespace

JoystickService::JoystickService()
{
    api_mutex_ = xSemaphoreCreateMutexStatic(&api_mutex_storage_);
    status_mutex_ = xSemaphoreCreateMutexStatic(&status_mutex_storage_);
    exit_sem_ = xSemaphoreCreateBinaryStatic(&exit_sem_storage_);
}

JoystickService::~JoystickService()
{
    // Не разрушаем объект с живой задачей: ждём подтверждения, как AxisMotor.
    while (stop() == ESP_ERR_TIMEOUT) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t JoystickService::start(const board::JoystickPins &pins,
                                 input::ButtonListener *listener)
{
    if (api_mutex_ == nullptr || status_mutex_ == nullptr ||
        exit_sem_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    lockApi();
    if (lifecycle_ != Lifecycle::Stopped) {
        unlockApi();
        return ESP_ERR_INVALID_STATE;
    }

    input::JoystickConfig config = makeConfig();
    const input::SampleGateConfig gate_config = makeGateConfig();
    const input::ButtonConfig button_config = makeButtonConfig();
    if (!input::isValid(config) || !gate_.isValid(gate_config) ||
        !debouncer_.isValid(button_config)) {
        unlockApi();
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = adc_.init(pins);
    if (err != ESP_OK) {
        const esp_err_t result = startFailure(err);
        unlockApi();
        return result;
    }

    err = button_.init(pins);
    if (err != ESP_OK) {
        const esp_err_t result = startFailure(err);
        unlockApi();
        return result;
    }

#if CONFIG_JOY_CALIBRATE_ON_BOOT
    if (!calibrateCenters(config)) {
        const esp_err_t result = startFailure(ESP_ERR_INVALID_STATE);
        unlockApi();
        return result;
    }
#endif

    if (!processor_.configure(config)) {
        const esp_err_t result = startFailure(ESP_ERR_INVALID_ARG);
        unlockApi();
        return result;
    }
    if (!gate_.configure(gate_config) || !debouncer_.configure(button_config)) {
        const esp_err_t result = startFailure(ESP_ERR_INVALID_ARG);
        unlockApi();
        return result;
    }

    const std::int64_t now_us = esp_timer_get_time();
    const auto now_ms = static_cast<std::uint32_t>(now_us / 1000);
    debouncer_.reset(button_.isPressed(), now_ms);

    local_ = JoystickStatus{};
    local_.calibrated = true;
    local_.stick = processor_.state();
    local_.neutral = local_.stick.neutral;

    period_ticks_ = samplePeriodTicks();
    listener_ = listener;

    (void)xSemaphoreTake(exit_sem_, 0);
    publishStatus();

    const BaseType_t created = xTaskCreatePinnedToCore(
        taskEntry, "joystick", CONFIG_JOY_TASK_STACK, this,
        CONFIG_JOY_TASK_PRIORITY, &task_, CONFIG_JOY_TASK_CORE);
    if (created != pdPASS) {
        const esp_err_t result = startFailure(ESP_ERR_NO_MEM);
        unlockApi();
        return result;
    }

    lifecycle_ = Lifecycle::Running;
    unlockApi();
    ESP_LOGI(kTag, "joystick polling started");
    return ESP_OK;
}

esp_err_t JoystickService::stop()
{
    if (api_mutex_ == nullptr) {
        return ESP_OK;
    }

    lockApi();
    if (lifecycle_ == Lifecycle::Stopped) {
        unlockApi();
        return ESP_OK;
    }

    // Уведомление отправляется только при переходе Running -> Stopping.
    // Повторный stop() после таймаута только ждёт подтверждения: task_
    // может содержать устаревший handle уже завершившейся задачи.
    if (lifecycle_ == Lifecycle::Running) {
        lifecycle_ = Lifecycle::Stopping;
        if (task_ != nullptr) {
            (void)xTaskNotifyGive(task_);
        }
    }

    const TickType_t timeout = timeoutTicks(CONFIG_JOY_STOP_TIMEOUT_MS);
    if (task_ != nullptr && xSemaphoreTake(exit_sem_, timeout) != pdTRUE) {
        unlockApi();
        ESP_LOGE(kTag, "task stop confirmation timeout");
        return ESP_ERR_TIMEOUT;
    }

    task_ = nullptr;
    listener_ = nullptr;
    adc_.deinit();
    button_.deinit();
    lifecycle_ = Lifecycle::Stopped;

    if (status_mutex_ != nullptr) {
        (void)xSemaphoreTake(status_mutex_, portMAX_DELAY);
        published_.sample_valid = false;
        published_.output_valid = false;
        published_.neutral = published_.stick.neutral;
        (void)xSemaphoreGive(status_mutex_);
    }
    unlockApi();

    ESP_LOGI(kTag, "joystick polling stopped");
    return ESP_OK;
}

JoystickStatus JoystickService::status() const
{
    if (status_mutex_ == nullptr) {
        return JoystickStatus{};
    }
    (void)xSemaphoreTake(status_mutex_, portMAX_DELAY);
    const JoystickStatus snapshot = published_;
    (void)xSemaphoreGive(status_mutex_);
    return snapshot;
}

void JoystickService::taskEntry(void *arg)
{
    static_cast<JoystickService *>(arg)->run();
}

void JoystickService::run()
{
    const TickType_t period = period_ticks_ == 0 ? 1 : period_ticks_;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        if (xTaskDelayUntil(&last_wake, period) == pdFALSE) {
            // Существенное отставание: базу переустанавливаем, серию
            // быстрых чтений не исполняем.
            last_wake = xTaskGetTickCount();
        }
        if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
            break;
        }

        const std::int64_t now_us = esp_timer_get_time();
        const auto now_ms = static_cast<std::uint32_t>(now_us / 1000);

        // Кнопка обрабатывается всегда, независимо от результата ADC.
        const input::ButtonEvents events =
            debouncer_.update(button_.isPressed(), now_ms);
        local_.button_events += events.count;

        const input::RawSample raw = adc_.read();
        if (raw.valid) {
            local_.last_valid_us = static_cast<std::uint64_t>(now_us);
        } else {
            ++local_.read_errors;
        }

        const input::SampleGateAction action = gate_.update(raw.valid);
        switch (action) {
            case input::SampleGateAction::Process:
                local_.stick = processor_.update(raw);
                local_.output_valid = true;
                break;
            case input::SampleGateAction::RecoverAndProcess:
                // resetWithSample сам считает фильтр, нормализацию,
                // magnitude и состояние нейтрали.
                local_.stick = processor_.resetWithSample(raw);
                local_.output_valid = true;
                break;
            case input::SampleGateAction::Ignore:
                local_.output_valid = false;
                break;
        }

        local_.raw = raw;
        local_.sample_valid = raw.valid;
        local_.read_fault = gate_.faulted();
        local_.neutral = local_.stick.neutral;

        publishStatus();

        for (std::uint8_t i = 0; i < events.count; ++i) {
            if (listener_ != nullptr) {
                listener_->onButtonEvent(events.values[i]);
            }
        }
    }

    (void)xSemaphoreGive(exit_sem_);
    vTaskDelete(nullptr);
}

void JoystickService::publishStatus()
{
    if (status_mutex_ == nullptr) {
        return;
    }
    (void)xSemaphoreTake(status_mutex_, portMAX_DELAY);
    published_ = local_;
    (void)xSemaphoreGive(status_mutex_);
}

void JoystickService::lockApi() const
{
    (void)xSemaphoreTake(api_mutex_, portMAX_DELAY);
}

void JoystickService::unlockApi() const
{
    (void)xSemaphoreGive(api_mutex_);
}

bool JoystickService::calibrateCenters(input::JoystickConfig &config)
{
    input::CenterCalibrator calibrator;
    if (!calibrator.configure(
            static_cast<std::size_t>(CONFIG_JOY_CALIBRATION_SAMPLES),
            static_cast<float>(CONFIG_JOY_CALIBRATION_NOISE_LIMIT),
            static_cast<float>(CONFIG_JOY_CENTER_MIN),
            static_cast<float>(CONFIG_JOY_CENTER_MAX))) {
        return false;
    }

    const TickType_t period = samplePeriodTicks();
    const TickType_t timeout =
        timeoutTicks(static_cast<std::uint32_t>(CONFIG_JOY_CALIBRATION_TIMEOUT_MS));
    const TickType_t started = xTaskGetTickCount();
    TickType_t last_wake = started;

    while ((xTaskGetTickCount() - started) < timeout) {
        if (xTaskDelayUntil(&last_wake, period) == pdFALSE) {
            last_wake = xTaskGetTickCount();
        }
        // Ожидание могло выйти за timeout: новые отсчёты после истечения
        // срока не читаются и не принимаются.
        if ((xTaskGetTickCount() - started) >= timeout) {
            break;
        }

        const input::RawSample raw = adc_.read();
        if (raw.valid) {
            (void)calibrator.add(raw);
        }

        if (!calibrator.ready()) {
            continue;
        }
        if (calibrator.stable()) {
            config.axis[0].center = calibrator.center(input::StickAxis::X);
            config.axis[1].center = calibrator.center(input::StickAxis::Y);
            ESP_LOGI(kTag, "center calibrated: x=%.1f y=%.1f",
                     static_cast<double>(config.axis[0].center),
                     static_cast<double>(config.axis[1].center));
            return true;
        }
        calibrator.reset();
    }

    ESP_LOGE(kTag, "calibration timeout: release the stick and reboot");
    return false;
}

esp_err_t JoystickService::startFailure(esp_err_t error)
{
    adc_.deinit();
    button_.deinit();
    task_ = nullptr;
    listener_ = nullptr;
    lifecycle_ = Lifecycle::Stopped;

    if (status_mutex_ != nullptr) {
        (void)xSemaphoreTake(status_mutex_, portMAX_DELAY);
        published_ = JoystickStatus{};
        (void)xSemaphoreGive(status_mutex_);
    }
    return error;
}

}  // namespace input_hal
