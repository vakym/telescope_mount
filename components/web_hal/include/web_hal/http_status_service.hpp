/**
 * @file http_status_service.hpp
 * @brief Локальный HTTP-сервер состояния и его lifecycle.
 *
 * Контракт:
 *  - единственный владелец httpd_handle_t — этот объект, и только его
 *    задача-супервизор вызывает httpd_start()/httpd_stop();
 *  - сервер запускается только после подтверждённого события AP_START
 *    (RadioState::Running) и останавливается при уходе радио из Running;
 *  - состояние `Stopped` достигается только при подтверждённом завершении:
 *    в ESP-IDF 6.1 код ESP_FAIL от httpd_stop() означает, что сервер жив;
 *  - HTTP-слой только читает StateHub и AccessPointStatus. Моторных вызовов,
 *    записи в NVS/flash и изменения режима радио здесь нет.
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "esp_err.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "state_hal/state_hub.hpp"
#include "web/server_lifecycle.hpp"
#include "wifi_hal/wifi_ap_service.hpp"

namespace web_hal {

/// Снимок счётчиков для существующего heartbeat-лога main.
struct HttpStatusCounters {
    web::ServerState lifecycle = web::ServerState::Stopped;
    bool handle_present = false;

    std::uint32_t requests_total = 0;
    std::uint32_t page_requests = 0;
    std::uint32_t state_requests = 0;
    std::uint32_t request_failures = 0;
    std::uint32_t serialize_overflows = 0;

    std::uint32_t starts = 0;
    std::uint32_t stops = 0;
    std::uint32_t start_failures = 0;
    std::uint32_t stop_unconfirmed = 0;

    esp_err_t last_error = ESP_OK;
};

class HttpStatusService {
public:
    HttpStatusService(state_hal::StateHub &hub,
                      wifi_hal::WifiApService &access_point);
    ~HttpStatusService();

    HttpStatusService(const HttpStatusService &) = delete;
    HttpStatusService &operator=(const HttpStatusService &) = delete;

    /// Запускает задачу-супервизор. Ожидания Wi-Fi нет.
    esp_err_t start();

    /// Останавливает задачу; сервер закрывается в её финализации.
    esp_err_t stop();

    HttpStatusCounters counters() const;

private:
    static void taskEntry(void *arg);
    void run();

    bool retryElapsed(TickType_t now) const noexcept;
    void doStart();
    void doStop();
    void publishLifecycle() noexcept;

    static esp_err_t pageHandler(httpd_req_t *req);
    static esp_err_t stateHandler(httpd_req_t *req);
    static esp_err_t notFoundHandler(httpd_req_t *req, httpd_err_code_t error);
    static esp_err_t methodNotAllowedHandler(httpd_req_t *req,
                                             httpd_err_code_t error);

    state_hal::StateHub &hub_;
    wifi_hal::WifiApService &access_point_;

    /// Пишет и читает только задача-супервизор.
    web::ServerLifecycle lifecycle_{};
    httpd_handle_t handle_ = nullptr;
    TickType_t retry_at_ = 0;
    bool retry_pending_ = false;

    StaticSemaphore_t exit_sem_storage_{};
    SemaphoreHandle_t exit_sem_ = nullptr;
    TaskHandle_t task_ = nullptr;

    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> task_exited_{false};

    std::atomic<web::ServerState> published_lifecycle_{web::ServerState::Stopped};
    std::atomic<bool> handle_present_{false};
    std::atomic<esp_err_t> last_error_{ESP_OK};

    std::atomic<std::uint32_t> requests_total_{0};
    std::atomic<std::uint32_t> page_requests_{0};
    std::atomic<std::uint32_t> state_requests_{0};
    std::atomic<std::uint32_t> request_failures_{0};
    std::atomic<std::uint32_t> serialize_overflows_{0};
    std::atomic<std::uint32_t> starts_{0};
    std::atomic<std::uint32_t> stops_{0};
    std::atomic<std::uint32_t> start_failures_{0};
    std::atomic<std::uint32_t> stop_unconfirmed_{0};
};

}  // namespace web_hal
