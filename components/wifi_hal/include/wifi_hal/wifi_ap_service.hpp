/**
 * @file wifi_ap_service.hpp
 * @brief Служба точки доступа: NVS-настройки, радио и переключение.
 *
 * Протокол завершения (план 0004, редакция 6):
 *  - окончательным подтверждением завершения задачи служит полученный
 *    exit_sem_, а не флаг task_exited_;
 *  - первая финализация каждого запуска всегда ждёт семафор;
 *  - признак уже выполненной финализации — task_ == nullptr;
 *  - задача никогда не берёт api_mutex_, который stop() удерживает во время
 *    ожидания.
 *
 * Служба не зависит от motor_hal, input и input_hal: она только управляет
 * радио и публикует состояние. Решение «когда переключать» принимает main.
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "wifi/ap_config.hpp"
#include "wifi/ap_lifecycle.hpp"
#include "wifi/toggle_admission.hpp"

namespace wifi_hal {

/// Снимок состояния для периодической диагностики.
struct AccessPointStatus {
    wifi::Lifecycle lifecycle = wifi::Lifecycle::Stopped;
    wifi::RadioState radio_state = wifi::RadioState::Unknown;
    wifi::Operation operation = wifi::Operation::None;
    bool configured_start_on_boot = true;
    bool task_exited = false;
    std::uint8_t pending_toggles = 0;
    char ssid[wifi::kSsidCapacity] = {};
    std::uint8_t channel = 0;
    std::uint8_t auth_mode = 0;  ///< 0 = открытая сеть
    std::uint8_t stations = 0;

    std::uint32_t accepted_toggles = 0;
    std::uint32_t rejected_not_running = 0;
    std::uint32_t rejected_unconfirmed = 0;
    std::uint32_t rejected_full = 0;
    std::uint32_t executed_toggles = 0;
    std::uint32_t failed_on_shutdown = 0;
    std::uint32_t failed_on_timeout = 0;
    std::uint32_t failed_on_error = 0;
    std::uint32_t dropped_on_shutdown = 0;
    std::uint32_t dropped_on_timeout = 0;
    std::uint32_t dropped_on_error = 0;

    std::uint32_t rejected_api_busy = 0;
    std::uint32_t rejected_state_busy = 0;
    std::uint32_t internal_errors = 0;
    std::uint32_t event_timeouts = 0;
    std::uint32_t operation_errors = 0;
    std::uint32_t resyncs = 0;
    std::uint32_t starts = 0;
    std::uint32_t stops = 0;
    esp_err_t last_error = ESP_OK;
    esp_err_t teardown_result = ESP_OK;
};

class WifiApService {
public:
    /// Создаёт статические объекты синхронизации.
    WifiApService();

    /// Требует согласованной остановки: при таймауте повторяет ожидание.
    ~WifiApService();

    WifiApService(const WifiApService &) = delete;
    WifiApService &operator=(const WifiApService &) = delete;

    /**
     * @brief Инициализация NVS, драйвера и запуск точки доступа.
     *
     * Синхронный. При неподтверждённом запуске возвращает ошибку и
     * выполняет безопасный rollback.
     */
    esp_err_t start();

    /// Полная остановка. Возвращает сохранённую ошибку teardown.
    esp_err_t stop();

    /// Полностью неблокирующая заявка. ESP_OK = заявка сохранена.
    esp_err_t requestToggle();

    AccessPointStatus status() const;

private:
    /// Итог ожидания подтверждения операции.
    enum class Supervision : std::uint8_t {
        Confirmed,
        TimedOut,
        Mismatch,
        Aborted,
    };

    /// RAII-блокировки: освобождают мьютекс на любом выходе.
    class ApiLock;
    class StateLock;

    static void taskEntry(void *arg);
    void run();
    static void eventHandler(void *arg, esp_event_base_t base, int32_t id,
                             void *data);
    void handleWifiEvent(int32_t id);

    esp_err_t initialize(const wifi::AccessPointSettings &settings);
    esp_err_t initNetwork();
    esp_err_t initDriver();
    esp_err_t registerHandler();
    esp_err_t applyConfig(const wifi::AccessPointSettings &settings);
    esp_err_t initialStart(bool start_on_boot);
    esp_err_t failStart(esp_err_t cause);
    esp_err_t teardown();
    esp_err_t finalizeStopLocked();

    void doToggleOnce();
    void doResyncOnce();
    Supervision supervise(wifi::RadioState expected);

    StaticSemaphore_t api_mutex_storage_{};
    SemaphoreHandle_t api_mutex_ = nullptr;
    StaticSemaphore_t state_mutex_storage_{};
    SemaphoreHandle_t state_mutex_ = nullptr;
    StaticSemaphore_t exit_sem_storage_{};
    SemaphoreHandle_t exit_sem_ = nullptr;

    /// Жизненный цикл: изменяется только start()/stop() под api_mutex_.
    wifi::Lifecycle lifecycle_ = wifi::Lifecycle::Stopped;
    std::atomic<wifi::Lifecycle> published_lifecycle_{wifi::Lifecycle::Stopped};

    TaskHandle_t task_ = nullptr;
    /// Публикуется задачей без мьютекса до выдачи exit_sem_.
    esp_err_t teardown_result_ = ESP_OK;
    std::atomic<bool> task_exited_{false};
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> resync_requested_{false};

    std::atomic<std::uint32_t> rejected_api_busy_{0};
    std::atomic<std::uint32_t> rejected_state_busy_{0};
    std::atomic<std::uint32_t> internal_errors_{0};

    /// Защищено state_mutex_.
    esp_err_t last_error_ = ESP_OK;
    bool configured_start_on_boot_ = true;
    char ssid_[wifi::kSsidCapacity] = {};
    std::uint8_t channel_ = 0;
    std::uint8_t auth_mode_ = 0;

    wifi::ApCoordinator coordinator_{};
    wifi::ToggleAdmission admission_{};

    esp_netif_t *netif_ = nullptr;
    esp_event_handler_instance_t handler_instance_ = nullptr;
    bool handler_registered_ = false;
    bool wifi_initialized_ = false;
};

}  // namespace wifi_hal
