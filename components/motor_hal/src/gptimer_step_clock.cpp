/**
 * @file gptimer_step_clock.cpp
 * @brief GPTimer с непрерывным счётчиком и абсолютными alarm.
 */

#include "motor_hal/gptimer_step_clock.hpp"

#include "esp_attr.h"
#include "esp_check.h"
#include "sdkconfig.h"

namespace motor_hal {
namespace {

constexpr const char *kTag = "gptimer_clock";
constexpr std::uint32_t kResolutionHz = 1'000'000;

}  // namespace

GptimerStepClock::~GptimerStepClock()
{
    (void)deinit();
}

esp_err_t GptimerStepClock::init(Handler handler, void *ctx)
{
    handler_ = handler;
    ctx_ = ctx;

    gptimer_config_t config{};
    config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    config.direction = GPTIMER_COUNT_UP;
    config.resolution_hz = kResolutionHz;
    config.intr_priority = CONFIG_MOTOR_STEP_TIMER_INTR_PRIORITY;
    config.flags.intr_shared = 0;

    ESP_RETURN_ON_ERROR(gptimer_new_timer(&config, &timer_), kTag, "gptimer_new_timer failed");

    gptimer_event_callbacks_t callbacks{};
    callbacks.on_alarm = &GptimerStepClock::onAlarm;
    ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(timer_, &callbacks, this),
                        kTag, "gptimer_register_event_callbacks failed");

    ESP_RETURN_ON_ERROR(gptimer_enable(timer_), kTag, "gptimer_enable failed");
    // Счётчик идёт непрерывно: используется и для шагов, и для удержания.
    ESP_RETURN_ON_ERROR(gptimer_start(timer_), kTag, "gptimer_start failed");
    return ESP_OK;
}

esp_err_t GptimerStepClock::deinit()
{
    if (timer_ == nullptr) {
        return ESP_OK;
    }
    (void)gptimer_stop(timer_);
    (void)gptimer_disable(timer_);
    const esp_err_t err = gptimer_del_timer(timer_);
    timer_ = nullptr;
    return err;
}

bool GptimerStepClock::armAt(std::uint64_t deadline_us)
{
    if (timer_ == nullptr) {
        return false;
    }

    gptimer_alarm_config_t alarm{};
    alarm.alarm_count = deadline_us;
    alarm.reload_count = 0;
    alarm.flags.auto_reload_on_alarm = 0;
    return gptimer_set_alarm_action(timer_, &alarm) == ESP_OK;
}

void GptimerStepClock::disarm()
{
    if (timer_ != nullptr) {
        // nullptr отключает alarm; счётчик продолжает идти.
        (void)gptimer_set_alarm_action(timer_, nullptr);
    }
}

std::uint64_t GptimerStepClock::nowUs() const
{
    std::uint64_t value = 0;
    if (timer_ != nullptr) {
        (void)gptimer_get_raw_count(timer_, &value);
    }
    return value;
}

bool IRAM_ATTR GptimerStepClock::onAlarm(gptimer_handle_t timer,
                                         const gptimer_alarm_event_data_t *event,
                                         void *user_ctx)
{
    (void)timer;
    (void)event;
    auto *self = static_cast<GptimerStepClock *>(user_ctx);
    if (self == nullptr || self->handler_ == nullptr) {
        return false;
    }
    return self->handler_(self->ctx_);  // true пробуждает задачу оси
}

}  // namespace motor_hal
