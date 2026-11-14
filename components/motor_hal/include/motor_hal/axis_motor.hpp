/**
 * @file axis_motor.hpp
 * @brief Обвязка FreeRTOS вокруг motion::AxisExecutor.
 *
 * Класс не принимает решений о движении: он доставляет биты уведомления,
 * очередь команд и время GPTimer, а также публикует потокобезопасный снимок
 * статуса.
 */

#pragma once

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

    /// true = команда поставлена в очередь (не подтверждение приёма).
    bool post(const motion::AxisCommand &cmd);

    /// Приоритетный запрос без очереди.
    bool emergencyStop();

    motion::AxisStatus status() const;

    // motion::CommandSource. Вызывается только задачей этой оси:
    // очередь защищена от удаления жизненным циклом (api_mutex_).
    std::size_t take(motion::AxisCommand *out, std::size_t max_count) override;
    bool empty() const override;
    void clear() override;

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
};

}  // namespace motor_hal
