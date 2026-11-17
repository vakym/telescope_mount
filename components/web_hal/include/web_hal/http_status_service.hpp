/**
 * @file http_status_service.hpp
 * @brief Локальный HTTP-сервер состояния и его lifecycle.
 *
 * Контракт:
 *  - единственный владелец httpd_handle_t — этот объект;
 *  - решения принимает чистая обёртка web::ServerController: она применяет
 *    те же правила, что и автомат ServerLifecycle, и проверяется на хосте;
 *  - сервер запускается только при подтверждённом AP_START и
 *    останавливается при уходе радио из Running;
 *  - `Stopped` достигается только при подтверждённом останове; при
 *    неподтверждённом останове владение сохраняется, а объект нельзя
 *    уничтожать, пока сервер может вызывать обработчики с user_ctx = this;
 *  - финализация задачи-супервизора идёт через shutdownOnce(), а не отдельным
 *    вызовом httpd_stop();
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
#include "web/retry_gate.hpp"
#include "web/server_controller.hpp"
#include "web/shutdown_protocol.hpp"
#include "wifi_hal/wifi_ap_service.hpp"

namespace calibration_hal {
class CalibrationService;
}

namespace web_hal {

/// Снимок счётчиков для существующего heartbeat-лога main.
struct HttpStatusCounters {
    web::ServerState lifecycle = web::ServerState::Stopped;
    bool handle_present = false;

    std::uint32_t requests_total = 0;
    std::uint32_t page_requests = 0;
    std::uint32_t state_requests = 0;
    std::uint32_t calibration_requests = 0;
    std::uint32_t calibration_commands = 0;
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
                      wifi_hal::WifiApService &access_point,
                      calibration_hal::CalibrationService &calibration);
    ~HttpStatusService();

    HttpStatusService(const HttpStatusService &) = delete;
    HttpStatusService &operator=(const HttpStatusService &) = delete;

    /// Запускает задачу-супервизор. Ожидания Wi-Fi нет.
    esp_err_t start();

    /**
     * @brief Останавливает супервизор и сервер.
     *
     * Возвращает результат именно этого останова: `ESP_OK` — завершение
     * подтверждено, `ESP_ERR_TIMEOUT` — супервизор не подтвердил выход
     * (владение сохранено), `ESP_FAIL` — останов не подтверждён.
     */
    esp_err_t stop();

    HttpStatusCounters counters() const;

private:
    /// Мост к реальному серверу; доступ к handle_ только через владельца.
    class ServerPortImpl final : public web::ServerPort {
    public:
        explicit ServerPortImpl(HttpStatusService &owner) noexcept
            : owner_(owner)
        {
        }

        web::StartOutcome start() noexcept override
        {
            return owner_.portStart();
        }

        web::StopOutcome stop() noexcept override
        {
            return owner_.portStop();
        }

        bool hasServer() const noexcept override
        {
            return owner_.handle_ != nullptr;
        }

    private:
        HttpStatusService &owner_;
    };

    static void taskEntry(void *arg);
    void run();

    void superviseOnce();
    std::uint32_t retryIntervalTicks() const noexcept;
    void publishState() noexcept;

    web::StartOutcome portStart();
    web::StopOutcome portStop();

    static esp_err_t pageHandler(httpd_req_t *req);
    static esp_err_t stateHandler(httpd_req_t *req);
    static esp_err_t calibrationStatusHandler(httpd_req_t *req);
    /// Один обработчик на все POST-маршруты: тип команды определяется URI.
    static esp_err_t calibrationCommandHandler(httpd_req_t *req);
    esp_err_t sendCalibrationStatus(httpd_req_t *req);
    static esp_err_t notFoundHandler(httpd_req_t *req, httpd_err_code_t error);
    static esp_err_t methodNotAllowedHandler(httpd_req_t *req,
                                             httpd_err_code_t error);

    state_hal::StateHub &hub_;
    wifi_hal::WifiApService &access_point_;
    calibration_hal::CalibrationService &calibration_;

    ServerPortImpl port_;
    web::ServerController controller_;

    /// Пишет и читает только задача-супервизор.
    httpd_handle_t handle_ = nullptr;
    /// Каждая выполненная попытка заново отсчитывает интервал повтора.
    web::RetryGate retry_gate_{};

    StaticSemaphore_t exit_sem_storage_{};
    SemaphoreHandle_t exit_sem_ = nullptr;
    /// Задача подтверждает завершение и ждёт удаления владельцем.
    StaticSemaphore_t delete_gate_storage_{};
    SemaphoreHandle_t delete_gate_ = nullptr;
    TaskHandle_t task_ = nullptr;

    /// Сериализует start()/stop(); модель вызовов задана явно.
    StaticSemaphore_t call_mutex_storage_{};
    SemaphoreHandle_t call_mutex_ = nullptr;
    /// Только первый stop() разрешает уведомление задачи.
    web::ShutdownProtocol shutdown_protocol_{};

    std::atomic<bool> shutdown_requested_{false};
    /// Результат текущего останова, выставляется финализацией.
    std::atomic<esp_err_t> shutdown_result_{ESP_OK};

    std::atomic<web::ServerState> published_lifecycle_{web::ServerState::Stopped};
    std::atomic<bool> handle_present_{false};
    std::atomic<esp_err_t> last_error_{ESP_OK};

    std::atomic<std::uint32_t> requests_total_{0};
    std::atomic<std::uint32_t> page_requests_{0};
    std::atomic<std::uint32_t> state_requests_{0};
    std::atomic<std::uint32_t> calibration_requests_{0};
    std::atomic<std::uint32_t> calibration_commands_{0};
    std::atomic<std::uint32_t> request_failures_{0};
    std::atomic<std::uint32_t> serialize_overflows_{0};
    std::atomic<std::uint32_t> starts_{0};
    std::atomic<std::uint32_t> stops_{0};
    std::atomic<std::uint32_t> start_failures_{0};
    std::atomic<std::uint32_t> stop_unconfirmed_{0};
};

}  // namespace web_hal
