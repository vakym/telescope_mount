/**
 * @file main.cpp
 * @brief Точка входа прошивки контроллера монтировки телескопа.
 *
 * Этап 2: слой движения (motion + motor_hal). main только инициализирует
 * плату, собирает конфигурацию осей, запускает службу движения и, при
 * явно включённом самотесте, выполняет проверочный прогон на стенде.
 */

#include <cstdint>

#include "board/board.hpp"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_flash.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "input/button_debouncer.hpp"
#include "input_hal/joystick_service.hpp"
#include "motion/axis_config.hpp"
#include "motor_hal/motor_service.hpp"
#include "sdkconfig.h"

namespace {

constexpr const char *kTag = "telescope_mount";

/// Полушагов на оборот выходного вала 28BYJ-48 (номинал).
constexpr std::int64_t kHalfStepsPerRevolution = 4096;

const char *buttonEventName(input::ButtonEvent event)
{
    switch (event) {
        case input::ButtonEvent::Pressed:
            return "pressed";
        case input::ButtonEvent::Released:
            return "released";
        case input::ButtonEvent::Click:
            return "click";
        case input::ButtonEvent::LongPress:
            return "long_press";
        case input::ButtonEvent::None:
            break;
    }
    return "none";
}

/// Только логирование: конкретное действие кнопки пока не назначено.
class LoggingButtonListener final : public input::ButtonListener {
public:
    void onButtonEvent(input::ButtonEvent event) override
    {
        ESP_LOGI(kTag, "joystick button: %s", buttonEventName(event));
    }
};

void logChipInfo()
{
    esp_chip_info_t chip_info{};
    esp_chip_info(&chip_info);
    ESP_LOGI(kTag, "chip: %d core(s), silicon revision %d",
             chip_info.cores, chip_info.revision);

    std::uint32_t flash_size_bytes = 0;
    if (esp_flash_get_size(nullptr, &flash_size_bytes) == ESP_OK) {
        ESP_LOGI(kTag, "flash: %u MB",
                 static_cast<unsigned>(flash_size_bytes / (1024U * 1024U)));
    } else {
        ESP_LOGW(kTag, "flash: size unknown");
    }
}

motion::AxisConfig makeAxisConfig(motion::AxisId id)
{
    motion::AxisConfig cfg;
    cfg.id = id;
    cfg.min_phase_interval_us = CONFIG_MOTOR_MIN_STEP_INTERVAL_US;
    return cfg;
}

const char *axisName(motion::AxisId id)
{
    return id == motion::AxisId::Azimuth ? "azimuth" : "altitude";
}

bool isMoving(const motion::AxisStatus &status)
{
    return status.state == motion::AxisState::Accelerating ||
           status.state == motion::AxisState::Cruising ||
           status.state == motion::AxisState::Decelerating;
}

/**
 * @brief Ждёт завершения конкретного перемещения.
 *
 * Нулевой скорости недостаточно: сразу после постановки команды в очередь ось
 * ещё стоит, поэтому сначала подтверждаем начало движения, затем ждём
 * ожидаемую позицию без Fault.
 */
bool waitForMoveCompletion(motor_hal::MotorService &service, motion::AxisId id,
                           std::int64_t expected_position, std::uint32_t timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    const std::int64_t initial_position = service.status(id).position_step;
    bool accepted = false;

    while ((xTaskGetTickCount() - start) < timeout) {
        const motion::AxisStatus status = service.status(id);
        if (status.state == motion::AxisState::Fault || status.has_fault) {
            return false;
        }
        if (!accepted && (isMoving(status) || status.position_step != initial_position)) {
            accepted = true;
        }
        if (accepted && !isMoving(status) && status.rate_steps_s == 0.0F &&
            status.position_step == expected_position) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

void logJoystick(input_hal::JoystickService &service)
{
    const input_hal::JoystickStatus status = service.status();
    ESP_LOGI(kTag,
             "joystick: calibrated=%d sample=%d output=%d fault=%d neutral=%d "
             "x=%.3f y=%.3f raw=%u/%u last_valid_us=%llu errors=%lu events=%lu",
             status.calibrated ? 1 : 0, status.sample_valid ? 1 : 0,
             status.output_valid ? 1 : 0, status.read_fault ? 1 : 0,
             status.neutral ? 1 : 0,
             static_cast<double>(status.stick.normalized[0]),
             static_cast<double>(status.stick.normalized[1]),
             static_cast<unsigned>(status.raw.counts[0]),
             static_cast<unsigned>(status.raw.counts[1]),
             static_cast<unsigned long long>(status.last_valid_us),
             static_cast<unsigned long>(status.read_errors),
             static_cast<unsigned long>(status.button_events));
}

void logAxis(motor_hal::MotorService &service, motion::AxisId id)
{
    const motion::AxisStatus status = service.status(id);
    ESP_LOGI(kTag,
             "%s: state=%u pos=%lld rate=%.1f energized=%d trusted=%d "
             "late_corr=%llu late_replan=%llu stale=%llu rejected=%llu",
             axisName(id), static_cast<unsigned>(status.state),
             static_cast<long long>(status.position_step),
             static_cast<double>(status.rate_steps_s), status.energized ? 1 : 0,
             status.position_trusted ? 1 : 0,
             static_cast<unsigned long long>(status.late_corrections),
             static_cast<unsigned long long>(status.late_replans),
             static_cast<unsigned long long>(status.stale_events),
             static_cast<unsigned long long>(status.rejected_commands));
}

#if CONFIG_APP_MOTOR_SELFTEST

void runSelfTest(motor_hal::MotorService &service)
{
    ESP_LOGW(kTag, "self-test enabled: both axes will move");

    (void)service.enable(motion::AxisId::Azimuth);
    (void)service.enable(motion::AxisId::Altitude);

    // Азимут: два оборота с разгоном и плавным остановом.
    const std::int64_t azimuth_start = service.status(motion::AxisId::Azimuth).position_step;
    (void)service.moveSteps(motion::AxisId::Azimuth, 2 * kHalfStepsPerRevolution);
    if (!waitForMoveCompletion(service, motion::AxisId::Azimuth,
                               azimuth_start + 2 * kHalfStepsPerRevolution, 30000)) {
        ESP_LOGE(kTag, "azimuth move did not complete");
        (void)service.stopAxis(motion::AxisId::Azimuth, motion::StopMode::Immediate);
    }
    logAxis(service, motion::AxisId::Azimuth);

    // Высота: то же в обратном направлении.
    const std::int64_t altitude_start = service.status(motion::AxisId::Altitude).position_step;
    (void)service.moveSteps(motion::AxisId::Altitude, -2 * kHalfStepsPerRevolution);
    if (!waitForMoveCompletion(service, motion::AxisId::Altitude,
                               altitude_start - 2 * kHalfStepsPerRevolution, 30000)) {
        ESP_LOGE(kTag, "altitude move did not complete");
        (void)service.stopAxis(motion::AxisId::Altitude, motion::StopMode::Immediate);
    }
    logAxis(service, motion::AxisId::Altitude);

    // Одновременное движение и плавный останов.
    (void)service.setTargetRate(motion::AxisId::Azimuth, 400.0F);
    (void)service.setTargetRate(motion::AxisId::Altitude, -400.0F);
    vTaskDelay(pdMS_TO_TICKS(1000));
    (void)service.stopAll(motion::StopMode::Smooth);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Аварийный останов: задержка от запроса до снятия фаз.
    (void)service.setTargetRate(motion::AxisId::Azimuth, 600.0F);
    vTaskDelay(pdMS_TO_TICKS(300));
    const std::int64_t requested_us = esp_timer_get_time();
    (void)service.stopAll(motion::StopMode::Immediate);
    bool deenergized = false;
    std::int64_t released_us = 0;
    for (int i = 0; i < 200; ++i) {
        const motion::AxisStatus status = service.status(motion::AxisId::Azimuth);
        if (!isMoving(status) && !status.energized) {
            released_us = esp_timer_get_time();
            deenergized = true;
            break;
        }
        vTaskDelay(1);
    }
    if (deenergized) {
        ESP_LOGW(kTag, "emergency stop latency: %lld us",
                 static_cast<long long>(released_us - requested_us));
    } else {
        ESP_LOGE(kTag, "emergency stop: de-energizing not confirmed within 200 ms");
    }
    logAxis(service, motion::AxisId::Azimuth);
    logAxis(service, motion::AxisId::Altitude);
}

#endif  // CONFIG_APP_MOTOR_SELFTEST

}  // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "firmware starting");
    ESP_LOGI(kTag, "ESP-IDF version: %s", esp_get_idf_version());

    logChipInfo();

    ESP_ERROR_CHECK(board::init());

    motor_hal::MotorService service;
    const motion::AxisConfig configs[2] = {
        makeAxisConfig(motion::AxisId::Azimuth),
        makeAxisConfig(motion::AxisId::Altitude),
    };
    ESP_ERROR_CHECK(service.start(board::kMotorPins, configs));
    ESP_LOGI(kTag, "motion service started: half-step units, GPTimer step clock");

#if CONFIG_APP_MOTOR_SELFTEST
    runSelfTest(service);
#endif

    // Запуск после самотеста: синхронная калибровка до 10 с не задерживает
    // проверку приводов. Джойстик только публикует состояние, MotorService
    // здесь не используется.
    LoggingButtonListener button_listener;
    input_hal::JoystickService joystick;
    const esp_err_t joystick_err =
        joystick.start(board::kJoystickPins, &button_listener);
    if (joystick_err != ESP_OK) {
        ESP_LOGE(kTag, "joystick unavailable: %s",
                 esp_err_to_name(joystick_err));
    }

    while (true) {
        ESP_LOGI(kTag, "heartbeat");
        logAxis(service, motion::AxisId::Azimuth);
        logAxis(service, motion::AxisId::Altitude);
        logJoystick(joystick);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_APP_STATUS_PERIOD_MS));
    }
}
