/**
 * @file http_status_service.cpp
 * @brief Lifecycle локального HTTP-сервера, маршруты и счётчики.
 *
 * Владение:
 *  - handle_ изменяется только задачей-супервизором;
 *  - start()/stop() из другой задачи не касаются handle_, а только запускают
 *    или останавливают задачу;
 *  - обработчики не вызывают моторный слой и не удерживают мьютекс хаба во
 *    время отправки: копия снимается один раз в начале обработчика.
 *
 * Останов: `Stopped` достигается только при подтверждённом завершении. В
 * ESP-IDF 6.1 `httpd_stop()` возвращает `ESP_FAIL`, если управляющее
 * сообщение не отправлено, и при этом сервер продолжает работать; признаком
 * освобождения ресурсов служит только `ESP_OK`.
 */

#include "web_hal/http_status_service.hpp"

#include <cstddef>
#include <cstdint>

#include "esp_log.h"
#include "sdkconfig.h"
#include "web/state_json.hpp"

// Символы встроенного ресурса: файл копируется в образ через EMBED_FILES.
extern "C" {
extern const unsigned char _binary_index_html_start[];
extern const unsigned char _binary_index_html_end[];
}

#ifndef CONFIG_WEB_ENABLED
#error "web_hal must be built only when CONFIG_WEB_ENABLED is enabled"
#endif

namespace web_hal {
namespace {

constexpr const char *kTag = "web";

constexpr const char *kPageContentType = "text/html; charset=utf-8";
constexpr const char *kJsonContentType = "application/json; charset=utf-8";
constexpr const char *kTextContentType = "text/plain; charset=utf-8";
constexpr const char *kCacheControl = "Cache-Control";
constexpr const char *kNoStore = "no-store";
constexpr const char *kNotFoundBody = "not found";
constexpr const char *kMethodNotAllowedBody = "method not allowed";
constexpr const char *kOverflowBody = "{\"error\":\"serialize_overflow\"}";

/// esp_http_server резервирует три сокета сверх max_open_sockets.
static_assert(CONFIG_WEB_MAX_OPEN_SOCKETS + 3 <= CONFIG_LWIP_MAX_SOCKETS,
              "WEB_MAX_OPEN_SOCKETS is too large for CONFIG_LWIP_MAX_SOCKETS");

}  // namespace

HttpStatusService::HttpStatusService(state_hal::StateHub &hub,
                                     wifi_hal::WifiApService &access_point)
    : hub_(hub), access_point_(access_point), exit_sem_storage_{}
{
    exit_sem_ = xSemaphoreCreateBinaryStatic(&exit_sem_storage_);
}

HttpStatusService::~HttpStatusService()
{
    while (stop() == ESP_ERR_TIMEOUT) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t HttpStatusService::start()
{
    if (exit_sem_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    if (task_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    shutdown_requested_.store(false, std::memory_order_release);
    task_exited_.store(false, std::memory_order_release);
    retry_pending_ = false;
    while (xSemaphoreTake(exit_sem_, 0) == pdTRUE) {
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        taskEntry, "web_sup", CONFIG_WEB_SUPERVISOR_STACK, this,
        CONFIG_WEB_SUPERVISOR_PRIORITY, &task_, CONFIG_WEB_SUPERVISOR_CORE);
    if (created != pdPASS) {
        task_ = nullptr;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t HttpStatusService::stop()
{
    if (task_ == nullptr) {
        return ESP_OK;
    }

    shutdown_requested_.store(true, std::memory_order_release);
    (void)xTaskNotifyGive(task_);

    if (xSemaphoreTake(exit_sem_, pdMS_TO_TICKS(CONFIG_WEB_STOP_TIMEOUT_MS)) !=
        pdTRUE) {
        // Задача не подтвердила завершение; lifecycle остаётся прежним.
        return ESP_ERR_TIMEOUT;
    }
    task_ = nullptr;
    return last_error_.load(std::memory_order_acquire) == ESP_OK ? ESP_OK
                                                                : ESP_FAIL;
}

HttpStatusCounters HttpStatusService::counters() const
{
    HttpStatusCounters snapshot;
    snapshot.lifecycle = published_lifecycle_.load(std::memory_order_acquire);
    snapshot.handle_present = handle_present_.load(std::memory_order_acquire);
    snapshot.requests_total =
        requests_total_.load(std::memory_order_relaxed);
    snapshot.page_requests = page_requests_.load(std::memory_order_relaxed);
    snapshot.state_requests = state_requests_.load(std::memory_order_relaxed);
    snapshot.request_failures =
        request_failures_.load(std::memory_order_relaxed);
    snapshot.serialize_overflows =
        serialize_overflows_.load(std::memory_order_relaxed);
    snapshot.starts = starts_.load(std::memory_order_relaxed);
    snapshot.stops = stops_.load(std::memory_order_relaxed);
    snapshot.start_failures = start_failures_.load(std::memory_order_relaxed);
    snapshot.stop_unconfirmed =
        stop_unconfirmed_.load(std::memory_order_relaxed);
    snapshot.last_error = last_error_.load(std::memory_order_acquire);
    return snapshot;
}

void HttpStatusService::taskEntry(void *arg)
{
    static_cast<HttpStatusService *>(arg)->run();
}

void HttpStatusService::publishLifecycle() noexcept
{
    published_lifecycle_.store(lifecycle_.state(), std::memory_order_release);
}

bool HttpStatusService::retryElapsed(TickType_t now) const noexcept
{
    if (!retry_pending_) {
        return true;
    }
    const std::uint32_t interval_ms =
        lifecycle_.state() == web::ServerState::StartFailed
            ? CONFIG_WEB_START_RETRY_MS
            : CONFIG_WEB_STOP_RETRY_MS;
    return (now - retry_at_) >= pdMS_TO_TICKS(interval_ms);
}

void HttpStatusService::run()
{
    for (;;) {
        if (shutdown_requested_.load(std::memory_order_acquire)) {
            break;
        }

        const wifi_hal::AccessPointStatus status = access_point_.status();
        const bool ap_running = status.radio_state == wifi::RadioState::Running;
        const bool ap_stopping = status.operation == wifi::Operation::Stopping;
        const bool retry = retryElapsed(xTaskGetTickCount());

        const web::ServerAction action =
            lifecycle_.update(ap_running, ap_stopping, retry);
        if (action == web::ServerAction::Start) {
            retry_pending_ = false;
            starts_.fetch_add(1, std::memory_order_relaxed);
            doStart();
        } else if (action == web::ServerAction::Stop) {
            retry_pending_ = false;
            stops_.fetch_add(1, std::memory_order_relaxed);
            doStop();
        }
        publishLifecycle();

        (void)ulTaskNotifyTake(
            pdTRUE, pdMS_TO_TICKS(CONFIG_WEB_SUPERVISOR_PERIOD_MS));
    }

    // Терминальная финализация. Если останов уже признан недоказуемым,
    // handle не разыменовывается: второй экземпляр при этом не создаётся.
    if (handle_ != nullptr &&
        lifecycle_.state() != web::ServerState::StopFailed) {
        const esp_err_t err = httpd_stop(handle_);
        if (err == ESP_OK) {
            handle_ = nullptr;
            handle_present_.store(false, std::memory_order_release);
        } else {
            last_error_.store(err, std::memory_order_release);
            ESP_LOGE(kTag, "final httpd stop unconfirmed: %s",
                     esp_err_to_name(err));
        }
    }
    publishLifecycle();

    task_exited_.store(true, std::memory_order_release);
    (void)xSemaphoreGive(exit_sem_);
    vTaskDelete(nullptr);
}

void HttpStatusService::doStart()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = CONFIG_WEB_TASK_STACK;
    config.task_priority = CONFIG_WEB_TASK_PRIORITY;
    config.core_id = CONFIG_WEB_TASK_CORE;
    config.max_open_sockets =
        static_cast<std::uint16_t>(CONFIG_WEB_MAX_OPEN_SOCKETS);
    config.max_uri_handlers = 2;
    config.max_resp_headers = 4;
    config.backlog_conn = static_cast<std::uint16_t>(CONFIG_WEB_BACKLOG);
    config.recv_wait_timeout =
        static_cast<std::uint16_t>(CONFIG_WEB_RECV_TIMEOUT_S);
    config.send_wait_timeout =
        static_cast<std::uint16_t>(CONFIG_WEB_SEND_TIMEOUT_S);
    config.lru_purge_enable = true;
    config.keep_alive_enable = CONFIG_WEB_TCP_KEEPALIVE;

    httpd_handle_t handle = nullptr;
    esp_err_t err = httpd_start(&handle, &config);
    if (err != ESP_OK || handle == nullptr) {
        last_error_.store(err == ESP_OK ? ESP_FAIL : err,
                          std::memory_order_release);
        lifecycle_.startFailed();
        start_failures_.fetch_add(1, std::memory_order_relaxed);
        retry_pending_ = true;
        retry_at_ = xTaskGetTickCount();
        ESP_LOGE(kTag, "httpd start failed: %s", esp_err_to_name(err));
        return;
    }

    httpd_uri_t page = {};
    page.uri = "/";
    page.method = HTTP_GET;
    page.handler = &HttpStatusService::pageHandler;
    page.user_ctx = this;
    err = httpd_register_uri_handler(handle, &page);

    if (err == ESP_OK) {
        httpd_uri_t state_uri = {};
        state_uri.uri = "/api/state";
        state_uri.method = HTTP_GET;
        state_uri.handler = &HttpStatusService::stateHandler;
        state_uri.user_ctx = this;
        err = httpd_register_uri_handler(handle, &state_uri);
    }
    if (err == ESP_OK) {
        err = httpd_register_err_handler(handle, HTTPD_404_NOT_FOUND,
                                         &HttpStatusService::notFoundHandler);
    }
    if (err == ESP_OK) {
        err = httpd_register_err_handler(
            handle, HTTPD_405_METHOD_NOT_ALLOWED,
            &HttpStatusService::methodNotAllowedHandler);
    }

    if (err != ESP_OK) {
        // Откат частичного старта: сервер ещё не считается работающим.
        ESP_LOGE(kTag, "route registration failed: %s", esp_err_to_name(err));
        last_error_.store(err, std::memory_order_release);
        const esp_err_t stop_err = httpd_stop(handle);
        if (stop_err == ESP_OK) {
            lifecycle_.startFailed();
            start_failures_.fetch_add(1, std::memory_order_relaxed);
            retry_pending_ = true;
            retry_at_ = xTaskGetTickCount();
            return;
        }
        // Завершение отката не подтверждено: сервер мог остаться живым,
        // поэтому StartFailed недопустим. Handle сохраняется.
        handle_ = handle;
        handle_present_.store(true, std::memory_order_release);
        lifecycle_.startFailedWithLiveServer(stop_err == ESP_FAIL);
        stop_unconfirmed_.fetch_add(1, std::memory_order_relaxed);
        if (stop_err == ESP_FAIL) {
            retry_pending_ = true;
            retry_at_ = xTaskGetTickCount();
        } else {
            ESP_LOGE(kTag, "start rollback stop failed: %s",
                     esp_err_to_name(stop_err));
        }
        return;
    }

    handle_ = handle;
    handle_present_.store(true, std::memory_order_release);
    last_error_.store(ESP_OK, std::memory_order_release);
    lifecycle_.startSucceeded();
    ESP_LOGI(kTag, "http status page started: port=%u sockets=%u stack=%u",
             static_cast<unsigned>(config.server_port),
             static_cast<unsigned>(config.max_open_sockets),
             static_cast<unsigned>(config.stack_size));
}

void HttpStatusService::doStop()
{
    if (handle_ == nullptr) {
        lifecycle_.stopSucceeded();
        return;
    }

    const esp_err_t err = httpd_stop(handle_);
    if (err == ESP_OK) {
        handle_ = nullptr;
        handle_present_.store(false, std::memory_order_release);
        last_error_.store(ESP_OK, std::memory_order_release);
        lifecycle_.stopSucceeded();
        ESP_LOGI(kTag, "http status page stopped");
        return;
    }

    last_error_.store(err, std::memory_order_release);
    stop_unconfirmed_.fetch_add(1, std::memory_order_relaxed);
    // ESP_FAIL означает «сервер жив, handle валиден»: повтор допустим.
    lifecycle_.stopUnconfirmed(err == ESP_FAIL);
    if (err == ESP_FAIL) {
        retry_pending_ = true;
        retry_at_ = xTaskGetTickCount();
        ESP_LOGW(kTag, "httpd stop unconfirmed, retry scheduled");
    } else {
        // Неизвестный код: handle больше не используется.
        ESP_LOGE(kTag, "httpd stop failed with unexpected code: %s",
                 esp_err_to_name(err));
    }
}

esp_err_t HttpStatusService::pageHandler(httpd_req_t *req)
{
    auto *self = static_cast<HttpStatusService *>(req->user_ctx);
    if (self != nullptr) {
        self->requests_total_.fetch_add(1, std::memory_order_relaxed);
        self->page_requests_.fetch_add(1, std::memory_order_relaxed);
    }

    const std::size_t page_size = static_cast<std::size_t>(
        _binary_index_html_end - _binary_index_html_start);
    httpd_resp_set_type(req, kPageContentType);
    httpd_resp_set_hdr(req, kCacheControl, kNoStore);
    const esp_err_t err = httpd_resp_send(
        req, reinterpret_cast<const char *>(_binary_index_html_start),
        static_cast<ssize_t>(page_size));
    if (err != ESP_OK && self != nullptr) {
        self->request_failures_.fetch_add(1, std::memory_order_relaxed);
    }
    return err;
}

esp_err_t HttpStatusService::stateHandler(httpd_req_t *req)
{
    auto *self = static_cast<HttpStatusService *>(req->user_ctx);
    if (self == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    self->requests_total_.fetch_add(1, std::memory_order_relaxed);
    self->state_requests_.fetch_add(1, std::memory_order_relaxed);

    // Один согласованный снимок. Мьютекс хаба удерживается только на копии,
    // сериализация и отправка выполняются после его освобождения.
    const state::SystemSnapshot snapshot = self->hub_.snapshot();

    web::SerializeOptions options;
    options.axis_stale_us = CONFIG_CTRL_AXIS_STALE_US;
    options.joystick_stale_us = CONFIG_CTRL_AXIS_STALE_US;
    options.control_stale_us = CONFIG_CTRL_AXIS_STALE_US;

    char buffer[web::kMaxStateJsonBytes];
    std::size_t written = 0;
    if (web::stateJson(snapshot, options, buffer, sizeof(buffer), &written) !=
        web::SerializeResult::Ok) {
        // Ошибка сериализации не превращается в частично успешный ответ.
        self->serialize_overflows_.fetch_add(1, std::memory_order_relaxed);
        self->request_failures_.fetch_add(1, std::memory_order_relaxed);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, kJsonContentType);
        httpd_resp_set_hdr(req, kCacheControl, kNoStore);
        return httpd_resp_send(req, kOverflowBody, HTTPD_RESP_USE_STRLEN);
    }

    httpd_resp_set_type(req, kJsonContentType);
    httpd_resp_set_hdr(req, kCacheControl, kNoStore);
    const esp_err_t err =
        httpd_resp_send(req, buffer, static_cast<ssize_t>(written));
    if (err != ESP_OK) {
        self->request_failures_.fetch_add(1, std::memory_order_relaxed);
    }
    return err;
}

esp_err_t HttpStatusService::notFoundHandler(httpd_req_t *req,
                                             httpd_err_code_t error)
{
    (void)error;
    httpd_resp_set_status(req, "404 Not Found");
    httpd_resp_set_type(req, kTextContentType);
    httpd_resp_set_hdr(req, kCacheControl, kNoStore);
    (void)httpd_resp_send(req, kNotFoundBody, HTTPD_RESP_USE_STRLEN);
    // Соединение закрывается, как и в обработчике по умолчанию.
    return ESP_FAIL;
}

esp_err_t HttpStatusService::methodNotAllowedHandler(httpd_req_t *req,
                                                     httpd_err_code_t error)
{
    (void)error;
    httpd_resp_set_status(req, "405 Method Not Allowed");
    httpd_resp_set_type(req, kTextContentType);
    httpd_resp_set_hdr(req, kCacheControl, kNoStore);
    (void)httpd_resp_send(req, kMethodNotAllowedBody, HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
}

}  // namespace web_hal
