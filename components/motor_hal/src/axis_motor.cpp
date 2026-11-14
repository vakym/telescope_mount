/**
 * @file axis_motor.cpp
 * @brief Задача оси и доставка событий исполнителю.
 */

#include "motor_hal/axis_motor.hpp"

#include <algorithm>

#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "sdkconfig.h"

namespace motor_hal {
namespace {

constexpr const char *kTag = "axis_motor";
constexpr std::uint32_t kInfiniteWait = 0xFFFFFFFFU;

}  // namespace

esp_err_t AxisMotor::start(motion::AxisPlanner &planner,
                           const board::CoilPins &pins,
                           const motion::AxisConfig &config)
{
    if (running_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(coil_.init(pins, config.pattern, config.invert_direction),
                        kTag, "coil init failed");
    ESP_RETURN_ON_ERROR(clock_.init(&AxisMotor::alarmTrampoline, this),
                        kTag, "step clock init failed");

    motion::AxisExecutor::Params params{};
    params.watchdog_us = CONFIG_MOTOR_EVENT_WATCHDOG_US;
    params.command_budget = CONFIG_MOTOR_CMD_BUDGET_PER_ITERATION;
    executor_.emplace(planner, clock_, coil_, *this, params);

    queue_ = xQueueCreate(CONFIG_MOTOR_CMD_QUEUE_LEN, sizeof(motion::AxisCommand));
    ESP_RETURN_ON_FALSE(queue_ != nullptr, ESP_ERR_NO_MEM, kTag, "queue create failed");

    mutex_ = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(mutex_ != nullptr, ESP_ERR_NO_MEM, kTag, "mutex create failed");

    const BaseType_t created = xTaskCreatePinnedToCore(&AxisMotor::taskEntry,
                                                       "motor_axis",
                                                       CONFIG_MOTOR_AXIS_TASK_STACK,
                                                       this,
                                                       CONFIG_MOTOR_AXIS_TASK_PRIORITY,
                                                       &task_handle_,
                                                       CONFIG_MOTOR_AXIS_TASK_CORE);
    ESP_RETURN_ON_FALSE(created == pdPASS, ESP_ERR_NO_MEM, kTag, "task create failed");

    running_ = true;
    return ESP_OK;
}

esp_err_t AxisMotor::stop()
{
    if (!running_) {
        return ESP_OK;
    }
    running_ = false;
    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }
    (void)clock_.deinit();
    if (queue_ != nullptr) {
        vQueueDelete(queue_);
        queue_ = nullptr;
    }
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
    executor_.reset();
    return ESP_OK;
}

bool AxisMotor::post(const motion::AxisCommand &cmd)
{
    if (!running_ || queue_ == nullptr || task_handle_ == nullptr) {
        return false;
    }
    if (xQueueSend(queue_, &cmd, 0) != pdTRUE) {
        return false;
    }
    (void)xTaskNotify(task_handle_, motion::CommandPending, eSetBits);
    return true;
}

bool AxisMotor::emergencyStop()
{
    if (!running_ || task_handle_ == nullptr) {
        return false;
    }
    (void)xTaskNotify(task_handle_, motion::EmergencyStop, eSetBits);
    return true;
}

motion::AxisStatus AxisMotor::status() const
{
    lock();
    motion::AxisStatus snapshot = executor_.has_value() ? executor_->status() : motion::AxisStatus{};
    unlock();
    return snapshot;
}

std::size_t AxisMotor::take(motion::AxisCommand *out, std::size_t max_count)
{
    std::size_t taken = 0;
    if (queue_ == nullptr) {
        return 0;
    }
    while (taken < max_count) {
        if (xQueueReceive(queue_, &out[taken], 0) != pdTRUE) {
            break;
        }
        ++taken;
    }
    return taken;
}

bool AxisMotor::empty() const
{
    return queue_ == nullptr || uxQueueMessagesWaiting(queue_) == 0;
}

void AxisMotor::clear()
{
    if (queue_ == nullptr) {
        return;
    }
    motion::AxisCommand dropped{};
    while (xQueueReceive(queue_, &dropped, 0) == pdTRUE) {
    }
}

void AxisMotor::lock() const
{
    if (mutex_ != nullptr && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    }
}

void AxisMotor::unlock() const
{
    if (mutex_ != nullptr && xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        (void)xSemaphoreGive(mutex_);
    }
}

bool IRAM_ATTR AxisMotor::alarmTrampoline(void *arg)
{
    auto *self = static_cast<AxisMotor *>(arg);
    if (self == nullptr || self->task_handle_ == nullptr) {
        return false;
    }
    BaseType_t awoken = pdFALSE;
    (void)xTaskNotifyFromISR(self->task_handle_, motion::TimerEvent, eSetBits, &awoken);
    return awoken == pdTRUE;  // драйвер GPTimer сам выполнит переключение контекста
}

void AxisMotor::taskEntry(void *arg)
{
    static_cast<AxisMotor *>(arg)->run();
}

void AxisMotor::run()
{
    for (;;) {
        std::uint32_t wait_us = kInfiniteWait;
        lock();
        if (executor_.has_value()) {
            wait_us = executor_->nextWaitUs();
        }
        unlock();

        TickType_t ticks = portMAX_DELAY;
        if (wait_us != kInfiniteWait) {
            const std::uint32_t wait_ms = (wait_us + 999U) / 1000U;
            ticks = pdMS_TO_TICKS(std::max<std::uint32_t>(wait_ms, 1U));
        }

        std::uint32_t bits = 0;
        (void)xTaskNotifyWait(0, 0xFFFFFFFFU, &bits, ticks);

        lock();
        if (executor_.has_value()) {
            executor_->onWakeup(bits);
        }
        unlock();
    }
}

}  // namespace motor_hal
