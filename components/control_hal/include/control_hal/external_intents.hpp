/**
 * @file external_intents.hpp
 * @brief Входной канал будущих Browser и Automation.
 *
 * Транспорт переносит ЦЕЛЫЕ намерения: атомарность приёма не разрушается
 * разделением каналов. При временном отказе атомарного приёма сообщение
 * остаётся в очереди и повторяется на следующем цикле; окончательные отказы
 * (старая сессия) наблюдаемы в счётчиках.
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "control/intent_inbox.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

namespace control_hal {

class ExternalIntents {
public:
    ExternalIntents();
    ~ExternalIntents();

    ExternalIntents(const ExternalIntents &) = delete;
    ExternalIntents &operator=(const ExternalIntents &) = delete;

    /// Неблокирующая: try-lock и нулевой таймаут очереди.
    /// ESP_OK — принято; ESP_ERR_TIMEOUT — занято; ESP_ERR_NO_MEM — полна.
    esp_err_t submit(const control::Intent &intent);

    /// Вызывается только управляющей задачей в начале цикла.
    std::size_t drainInto(control::IntentInbox &inbox, state::Micros now_us);

    void reset();

    struct Stats {
        std::uint32_t busy = 0;
        std::uint32_t full = 0;
        std::uint32_t rejected_old = 0;
        std::uint32_t deferred_atomic = 0;
    };

    Stats stats() const;

private:
    StaticSemaphore_t mutex_storage_{};
    SemaphoreHandle_t mutex_ = nullptr;
    QueueHandle_t queue_ = nullptr;
    std::atomic<std::uint32_t> busy_{0};
    std::atomic<std::uint32_t> full_{0};
    std::atomic<std::uint32_t> rejected_old_{0};
    std::atomic<std::uint32_t> deferred_atomic_{0};
};

}  // namespace control_hal
