/**
 * @file state_hub.cpp
 * @brief Публикация снимков под одним мьютексом.
 */

#include "state_hal/state_hub.hpp"

#include <cstdint>

#include "esp_timer.h"

namespace state_hal {
namespace {

state::Micros nowUs() noexcept
{
    return static_cast<state::Micros>(esp_timer_get_time());
}

}  // namespace

StateHub::StateHub()
{
    mutex_ = xSemaphoreCreateMutexStatic(&mutex_storage_);
}

bool StateHub::valid(motion::AxisId id) noexcept
{
    return static_cast<std::uint8_t>(id) <
           static_cast<std::uint8_t>(motion::AxisId::Count);
}

void StateHub::publishAxis(motion::AxisId id, const motion::AxisStatus &status,
                           state::Micros polled_us, std::uint32_t read_failures) noexcept
{
    if (mutex_ == nullptr || !valid(id)) {
        return;
    }
    (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    auto &axis = published_.axis[static_cast<std::size_t>(id)];
    axis.status = status;
    axis.read_failures = read_failures;
    axis.meta.polled_us = polled_us;
    axis.meta.published_us = nowUs();
    ++axis.meta.sequence;
    axis.meta.valid = true;
    (void)xSemaphoreGive(mutex_);
}

void StateHub::publishJoystick(const state::JoystickObservation &obs) noexcept
{
    if (mutex_ == nullptr) {
        return;
    }
    (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    published_.joystick = obs;
    published_.joystick.meta.published_us = nowUs();
    ++published_.joystick.meta.sequence;
    published_.joystick.meta.valid = true;
    (void)xSemaphoreGive(mutex_);
}

void StateHub::publishControl(const state::ControlObservation &obs) noexcept
{
    if (mutex_ == nullptr) {
        return;
    }
    (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    published_.control = obs;
    published_.control.meta.published_us = nowUs();
    ++published_.control.meta.sequence;
    published_.control.meta.valid = true;
    (void)xSemaphoreGive(mutex_);
}

void StateHub::invalidateAxis(motion::AxisId id) noexcept
{
    if (mutex_ == nullptr || !valid(id)) {
        return;
    }
    (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    auto &axis = published_.axis[static_cast<std::size_t>(id)];
    axis.meta.valid = false;
    ++axis.meta.sequence;
    (void)xSemaphoreGive(mutex_);
}

void StateHub::invalidateJoystick() noexcept
{
    if (mutex_ == nullptr) {
        return;
    }
    (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    published_.joystick.meta.valid = false;
    ++published_.joystick.meta.sequence;
    (void)xSemaphoreGive(mutex_);
}

state::SystemSnapshot StateHub::snapshot() const noexcept
{
    if (mutex_ == nullptr) {
        return state::SystemSnapshot{};
    }
    (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    state::SystemSnapshot copy = published_;
    (void)xSemaphoreGive(mutex_);
    copy.generated_us = nowUs();
    return copy;
}

}  // namespace state_hal
