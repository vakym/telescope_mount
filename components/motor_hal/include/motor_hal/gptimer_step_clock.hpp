/**
 * @file gptimer_step_clock.hpp
 * @brief Одноразовые alarm на абсолютные дедлайны поверх GPTimer.
 *
 * Счётчик работает непрерывно с момента init() и не сбрасывается на каждом
 * шаге. Взвод выполняется gptimer_set_alarm_action() с абсолютным
 * alarm_count; disarm() отключает ожидающий alarm, но не останавливает
 * счётчик — он же используется для отсчёта удержания.
 */

#pragma once

#include <cstdint>

#include "driver/gptimer.h"
#include "esp_err.h"
#include "motion/executor_io.hpp"

namespace motor_hal {

class GptimerStepClock final : public motion::AlarmSink {
public:
    /// Возвращает true, если разбудет задача с более высоким приоритетом.
    using Handler = bool (*)(void *ctx);

    GptimerStepClock() = default;
    ~GptimerStepClock() override;

    GptimerStepClock(const GptimerStepClock &) = delete;
    GptimerStepClock &operator=(const GptimerStepClock &) = delete;

    esp_err_t init(Handler handler, void *ctx);
    esp_err_t deinit();

    bool armAt(std::uint64_t deadline_us) override;
    void disarm() override;
    std::uint64_t nowUs() const override;

private:
    static bool onAlarm(gptimer_handle_t timer,
                        const gptimer_alarm_event_data_t *event,
                        void *user_ctx);

    gptimer_handle_t timer_ = nullptr;
    Handler handler_ = nullptr;
    void *ctx_ = nullptr;
};

}  // namespace motor_hal
