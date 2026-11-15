/**
 * @file external_intents.cpp
 * @brief Очередь внешних намерений с повторным приёмом при занятости.
 */

#include "control_hal/external_intents.hpp"

namespace control_hal {
namespace {

constexpr std::size_t kQueueDepth = state::kSourceCount * 2;

}  // namespace

ExternalIntents::ExternalIntents()
{
    mutex_ = xSemaphoreCreateMutexStatic(&mutex_storage_);
    queue_ = xQueueCreate(kQueueDepth, sizeof(control::Intent));
}

ExternalIntents::~ExternalIntents()
{
    if (queue_ != nullptr) {
        vQueueDelete(queue_);
        queue_ = nullptr;
    }
}

esp_err_t ExternalIntents::submit(const control::Intent &intent)
{
    if (mutex_ == nullptr || queue_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(mutex_, 0) != pdTRUE) {
        busy_.fetch_add(1, std::memory_order_relaxed);
        return ESP_ERR_TIMEOUT;
    }
    const BaseType_t sent = xQueueSend(queue_, &intent, 0);
    (void)xSemaphoreGive(mutex_);
    if (sent != pdTRUE) {
        full_.fetch_add(1, std::memory_order_relaxed);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

std::size_t ExternalIntents::drainInto(control::IntentInbox &inbox,
                                       state::Micros now_us)
{
    if (queue_ == nullptr) {
        return 0;
    }
    std::size_t accepted = 0;
    for (;;) {
        control::Intent intent{};
        if (xQueuePeek(queue_, &intent, 0) != pdTRUE) {
            break;
        }
        control::DenyReason deny = control::DenyReason::None;
        if (inbox.submit(intent, now_us, deny)) {
            control::Intent dropped{};
            (void)xQueueReceive(queue_, &dropped, 0);
            ++accepted;
            continue;
        }
        if (deny == control::DenyReason::Busy) {
            // Сообщение сохраняется и повторяется на следующем цикле.
            deferred_atomic_.fetch_add(1, std::memory_order_relaxed);
            break;
        }
        control::Intent dropped{};
        (void)xQueueReceive(queue_, &dropped, 0);
        rejected_old_.fetch_add(1, std::memory_order_relaxed);
    }
    return accepted;
}

void ExternalIntents::reset()
{
    if (queue_ != nullptr) {
        (void)xQueueReset(queue_);
    }
}

ExternalIntents::Stats ExternalIntents::stats() const
{
    Stats out{};
    out.busy = busy_.load(std::memory_order_relaxed);
    out.full = full_.load(std::memory_order_relaxed);
    out.rejected_old = rejected_old_.load(std::memory_order_relaxed);
    out.deferred_atomic = deferred_atomic_.load(std::memory_order_relaxed);
    return out;
}

}  // namespace control_hal
