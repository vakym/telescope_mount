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

/// Перевод микросекунд в тики с округлением вверх и без переполнения.
TickType_t usToTicksCeil(std::uint32_t wait_us) noexcept
{
    constexpr std::uint64_t kUsPerSecond = 1'000'000ULL;
    const std::uint64_t ticks =
        (static_cast<std::uint64_t>(wait_us) * configTICK_RATE_HZ + kUsPerSecond - 1ULL) /
        kUsPerSecond;
    return static_cast<TickType_t>(ticks == 0ULL ? 1ULL : ticks);
}

}  // namespace

AxisMotor::AxisMotor()
{
    // Статическое хранилище: mutex существует с момента конструирования,
    // поэтому ленивое создание (и гонка двух первых start()) невозможно.
    api_mutex_ = xSemaphoreCreateMutexStatic(&api_mutex_storage_);
}

AxisMotor::~AxisMotor()
{
    if (api_mutex_ == nullptr) {
        return;
    }

    // Работающая задача хранит this. Пока она не подтвердила завершение,
    // разрушать объект нельзя: ждём, а не игнорируем таймаут.
    bool warned = false;
    for (;;) {
        const esp_err_t err = stop();
        if (err != ESP_ERR_TIMEOUT) {
            break;
        }
        if (!warned) {
            ESP_LOGE(kTag, "shutdown timeout: waiting for axis task before destruction");
            warned = true;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // api_mutex_ создан статически и не удаляется: он не переживает объект.
}

esp_err_t AxisMotor::start(motion::AxisPlanner &planner,
                           const board::CoilPins &pins,
                           const motion::AxisConfig &config)
{
    if (api_mutex_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    (void)xSemaphoreTake(api_mutex_, portMAX_DELAY);
    if (lifecycle_ != Lifecycle::Stopped) {
        (void)xSemaphoreGive(api_mutex_);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = coil_.init(pins, config.pattern, config.invert_direction);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "coil init failed");
        cleanupResources();
        (void)xSemaphoreGive(api_mutex_);
        return err;
    }

    err = clock_.init(&AxisMotor::alarmTrampoline, this);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "step clock init failed");
        cleanupResources();
        (void)xSemaphoreGive(api_mutex_);
        return err;
    }

    motion::AxisExecutor::Params params{};
    params.watchdog_us = CONFIG_MOTOR_EVENT_WATCHDOG_US;
    params.command_budget = CONFIG_MOTOR_CMD_BUDGET_PER_ITERATION;
    executor_.emplace(planner, clock_, coil_, *this, params);

    queue_ = xQueueCreate(CONFIG_MOTOR_CMD_QUEUE_LEN, sizeof(motion::AxisCommand));
    mutex_ = xSemaphoreCreateMutex();
    exit_sem_ = xSemaphoreCreateBinary();
    if (queue_ == nullptr || mutex_ == nullptr || exit_sem_ == nullptr) {
        ESP_LOGE(kTag, "resource allocation failed");
        cleanupResources();
        (void)xSemaphoreGive(api_mutex_);
        return ESP_ERR_NO_MEM;
    }

    const BaseType_t created = xTaskCreatePinnedToCore(&AxisMotor::taskEntry,
                                                       "motor_axis",
                                                       CONFIG_MOTOR_AXIS_TASK_STACK,
                                                       this,
                                                       CONFIG_MOTOR_AXIS_TASK_PRIORITY,
                                                       &task_handle_,
                                                       CONFIG_MOTOR_AXIS_TASK_CORE);
    if (created != pdPASS) {
        ESP_LOGE(kTag, "task create failed");
        task_handle_ = nullptr;
        cleanupResources();
        (void)xSemaphoreGive(api_mutex_);
        return ESP_ERR_NO_MEM;
    }

    lifecycle_ = Lifecycle::Running;
    (void)xSemaphoreGive(api_mutex_);
    return ESP_OK;
}

esp_err_t AxisMotor::stop()
{
    if (api_mutex_ == nullptr) {
        return ESP_OK;
    }

    (void)xSemaphoreTake(api_mutex_, portMAX_DELAY);
    if (lifecycle_ == Lifecycle::Stopped) {
        (void)xSemaphoreGive(api_mutex_);
        return ESP_OK;
    }

    if (lifecycle_ == Lifecycle::Running) {
        lifecycle_ = Lifecycle::Stopping;  // новые команды не принимаются
        if (task_handle_ != nullptr) {
            // Задача сама обесточит фазы (EmergencyStop), выйдет из цикла по
            // TaskExit и подтвердит завершение.
            (void)xTaskNotify(task_handle_, motion::EmergencyStop | motion::TaskExit, eSetBits);
        }
    }

    if (task_handle_ != nullptr) {
        if (exit_sem_ == nullptr ||
            xSemaphoreTake(exit_sem_, pdMS_TO_TICKS(1000)) != pdTRUE) {
            // Задача могла не завершиться: ресурсы освобождать нельзя,
            // состояние остаётся Stopping, повторный stop() может дождаться.
            ESP_LOGE(kTag, "axis task did not confirm shutdown");
            (void)xSemaphoreGive(api_mutex_);
            return ESP_ERR_TIMEOUT;
        }
        task_handle_ = nullptr;
    }

    cleanupResources();
    lifecycle_ = Lifecycle::Stopped;
    (void)xSemaphoreGive(api_mutex_);
    return ESP_OK;
}

void AxisMotor::cleanupResources()
{
    (void)clock_.deinit();  // ISR больше не может сработать
    coil_.release();        // фазы гарантированно сняты

    if (queue_ != nullptr) {
        vQueueDelete(queue_);
        queue_ = nullptr;
    }
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
    if (exit_sem_ != nullptr) {
        vSemaphoreDelete(exit_sem_);
        exit_sem_ = nullptr;
    }
    executor_.reset();
}

bool AxisMotor::post(const motion::AxisCommand &cmd)
{
    if (api_mutex_ == nullptr) {
        return false;
    }
    (void)xSemaphoreTake(api_mutex_, portMAX_DELAY);
    bool queued = false;
    if (lifecycle_ == Lifecycle::Running && queue_ != nullptr && task_handle_ != nullptr) {
        queued = (xQueueSend(queue_, &cmd, 0) == pdTRUE) &&
                 (xTaskNotify(task_handle_, motion::CommandPending, eSetBits) == pdPASS);
    }
    (void)xSemaphoreGive(api_mutex_);
    return queued;
}

bool AxisMotor::emergencyStop()
{
    if (api_mutex_ == nullptr) {
        return false;
    }
    (void)xSemaphoreTake(api_mutex_, portMAX_DELAY);
    bool notified = false;
    if (lifecycle_ == Lifecycle::Running && task_handle_ != nullptr) {
        notified = (xTaskNotify(task_handle_, motion::EmergencyStop, eSetBits) == pdPASS);
    }
    (void)xSemaphoreGive(api_mutex_);
    return notified;
}

motion::AxisStatus AxisMotor::status() const
{
    if (api_mutex_ == nullptr) {
        return motion::AxisStatus{};
    }
    (void)xSemaphoreTake(api_mutex_, portMAX_DELAY);
    lock();
    motion::AxisStatus snapshot = executor_.has_value() ? executor_->status() : motion::AxisStatus{};
    unlock();
    (void)xSemaphoreGive(api_mutex_);
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

        const TickType_t ticks =
            (wait_us == kInfiniteWait) ? portMAX_DELAY : usToTicksCeil(wait_us);

        std::uint32_t bits = 0;
        (void)xTaskNotifyWait(0, 0xFFFFFFFFU, &bits, ticks);

        lock();
        if (executor_.has_value()) {
            executor_->onWakeup(bits);
        }
        bool more_commands = executor_.has_value() && executor_->hasPendingCommands();
        unlock();

        if ((bits & motion::TaskExit) != 0U) {
            break;  // фазы уже сняты обработкой EmergencyStop
        }
        if (more_commands && task_handle_ != nullptr) {
            // Остаток очереди после исчерпания бюджета обрабатывается без
            // внешней отправки новой команды.
            (void)xTaskNotify(task_handle_, motion::CommandPending, eSetBits);
        }
    }

    if (exit_sem_ != nullptr) {
        (void)xSemaphoreGive(exit_sem_);
    }
    vTaskDelete(nullptr);
}

}  // namespace motor_hal
