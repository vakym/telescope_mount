/**
 * @file axis_motor.hpp
 * @brief Обвязка FreeRTOS вокруг motion::AxisExecutor.
 *
 * Класс не принимает решений о движении: он доставляет биты уведомления,
 * очередь команд и время GPTimer, а также публикует потокобезопасный снимок
 * статуса.
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "board/board.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "motion/axis_command.hpp"
#include "motion/axis_config.hpp"
#include "motion/axis_executor.hpp"
#include "motion/axis_planner.hpp"
#include "motion/executor_io.hpp"
#include "motor_hal/coil_driver.hpp"
#include "motor_hal/gptimer_step_clock.hpp"

namespace motor_hal {

/// Результат приёма команды. Для FIFO-команды accepted означает фактическое
/// присутствие в очереди, для Immediate — публикацию id в emergency-слоте.
struct CommandReceipt {
    bool accepted = false;
    bool notified = false;
    std::uint64_t command_id = 0;
    std::uint32_t epoch = 0;
};

class AxisMotor final : private motion::CommandSource {
public:
    /**
     * @brief Создаёт статический api_mutex_.
     *
     * Контракт владения: создание, start(), stop() и разрушение выполняет
     * один владелец (MotorService) до публикации объекта другим задачам.
     * Конкурентный первый start() не поддерживается; после публикации
     * потокобезопасны команды и статус.
     */
    AxisMotor();
    /// Требует подтверждённого shutdown: при таймауте ждёт, а не разрушает
    /// объект с работающей задачей.
    ~AxisMotor();

    AxisMotor(const AxisMotor &) = delete;
    AxisMotor &operator=(const AxisMotor &) = delete;

    esp_err_t start(motion::AxisPlanner &planner,
                    const board::CoilPins &pins,
                    const motion::AxisConfig &config);
    esp_err_t stop();

    /// Назначает новый ненулевой id, ставит команду в очередь и уведомляет
    /// задачу. Входное значение поля id игнорируется.
    CommandReceipt post(motion::AxisCommand cmd);

    /// Приоритетный запрос без очереди: id публикуется в слоте.
    CommandReceipt emergencyStop();

    motion::AxisStatus status() const;

    /// Диагностика канала приоритетного останова.
    std::uint32_t emergencySlotBusy() const noexcept
    {
        return emergency_slot_busy_.load(std::memory_order_relaxed);
    }
    std::uint32_t notifyFailures() const noexcept
    {
        return notify_failures_.load(std::memory_order_relaxed);
    }

    // motion::CommandSource. Вызывается только задачей этой оси:
    // очередь защищена от удаления жизненным циклом (api_mutex_).
    std::size_t take(motion::AxisCommand *out, std::size_t max_count) override;
    bool empty() const override;
    void clear() override;
    std::uint64_t takeEmergencyId() override;

private:
    /// Явный жизненный цикл: таймаут подтверждения оставляет ось в Stopping.
    enum class Lifecycle : std::uint8_t { Stopped, Running, Stopping };

    static void taskEntry(void *arg);
    static bool IRAM_ATTR alarmTrampoline(void *arg);
    void run();
    void lock() const;
    void unlock() const;

    /// Освобождает всё, что успел создать start(); идемпотентен.
    void cleanupResources();

    /// Следующий идентификатор команды; 0 — исчерпан, не переиспользуем.
    std::uint64_t nextId() noexcept;

    GptimerStepClock clock_{};
    CoilDriver coil_{};
    std::optional<motion::AxisExecutor> executor_{};
    QueueHandle_t queue_ = nullptr;
    SemaphoreHandle_t mutex_ = nullptr;
    SemaphoreHandle_t exit_sem_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;

    /// Сериализует публичные методы и освобождение ресурсов. Создаётся в
    /// конструкторе из статического хранилища и никогда не удаляется, иначе
    /// проверка lifecycle_ и последующее использование queue_ были бы гонкой.
    StaticSemaphore_t api_mutex_storage_{};
    SemaphoreHandle_t api_mutex_ = nullptr;
    Lifecycle lifecycle_ = Lifecycle::Stopped;

    /// Назначается в единственной точке отправки и не сбрасывается при
    /// перезапуске верхнего слоя; эпоха меняется только на start().
    std::uint64_t next_id_ = 1;
    std::uint32_t epoch_ = 0;
    std::atomic<std::uint64_t> pending_emergency_id_{0};
    std::atomic<std::uint32_t> emergency_slot_busy_{0};
    std::atomic<std::uint32_t> notify_failures_{0};
};

}  // namespace motor_hal
