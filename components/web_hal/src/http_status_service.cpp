/**
 * @file http_status_service.cpp
 * @brief Lifecycle локального HTTP-сервера, маршруты и счётчики.
 *
 * Владение:
 *  - handle_ изменяется только внутри portStart()/portStop(), вызываемых из
 *    задачи-супервизора через web::ServerController;
 *  - start()/stop() из другой задачи не касаются handle_;
 *  - обработчики не вызывают моторный слой и не удерживают мьютекс хаба во
 *    время отправки: копия снимается один раз в начале обработчика.
 *
 * Останов: `Stopped` достигается только при подтверждённом завершении. В
 * ESP-IDF 6.1 `httpd_stop()` возвращает `ESP_FAIL`, если управляющее
 * сообщение не отправлено, и при этом сервер продолжает работать; признаком
 * освобождения ресурсов служит только `ESP_OK`. Поэтому финализация задачи
 * идёт через `ServerController::shutdownOnce()`: при неподтверждённом
 * останове владение сохраняется, и объект не уничтожается.
 */

#include "web_hal/http_status_service.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "calibration_hal/calibration_service.hpp"
#include "esp_log.h"
#include "sdkconfig.h"
#include "web/calibration_command.hpp"
#include "web/calibration_json.hpp"
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

/// Тело команды калибровки ограничено сверху: больше — не наш клиент.
constexpr std::size_t kMaxCommandBodyBytes = 256;
constexpr const char *kBadRequestBody = "{\"error\":\"invalid_request\"}";
constexpr const char *kTooLargeBody = "{\"error\":\"payload_too_large\"}";

web::CalibrationView toView(const calibration_hal::CalibrationStatus &status)
{
    web::CalibrationView view{};
    view.state = status.state;
    view.error = status.error;
    view.invalid_reason = status.invalid_reason;
    view.session_id = status.session_id;
    view.revision = status.revision;
    view.motion_generation = status.motion_generation;
    view.axis = status.axis;
    view.direction = status.direction;
    view.revolutions = status.revolutions;
    view.repeats = status.repeats;
    view.measurement_count = status.measurement_count;
    view.measurements = status.measurements;
    view.result = status.result;
    view.mark_recheck_required = status.mark_recheck_required;
    view.hold_active = status.hold_active;
    view.input_ready = status.input_ready;
    view.flash_window_active = status.flash_window_active;
    view.lease_remaining_ms = status.lease_remaining_ms;
    for (std::size_t i = 0; i < 2; ++i) {
        view.scale[i] = status.axes[i].scale;
        view.nominal_half_steps_per_revolution[i] =
            status.axes[i].nominal_half_steps_per_revolution;
    }
    return view;
}

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

/// Таблица POST-маршрутов калибровки: URI определяет тип команды.
struct PostRoute {
    const char *uri = nullptr;
    web::CalibrationCommandKind kind = web::CalibrationCommandKind::Stop;
};

constexpr PostRoute kCalibrationPosts[] = {
    {"/api/calibration/session", web::CalibrationCommandKind::CreateSession},
    {"/api/calibration/move", web::CalibrationCommandKind::Move},
    {"/api/calibration/stop", web::CalibrationCommandKind::Stop},
    {"/api/calibration/confirm-start", web::CalibrationCommandKind::ConfirmStart},
    {"/api/calibration/confirm-end", web::CalibrationCommandKind::ConfirmEnd},
    {"/api/calibration/resume", web::CalibrationCommandKind::Resume},
    {"/api/calibration/abandon", web::CalibrationCommandKind::Abandon},
    {"/api/calibration/measurements/delete", web::CalibrationCommandKind::DeleteMeasurement},
    {"/api/calibration/repeat", web::CalibrationCommandKind::AddRepeat},
    {"/api/calibration/apply", web::CalibrationCommandKind::Apply},
    {"/api/calibration/close", web::CalibrationCommandKind::Close},
    {"/api/calibration/reset", web::CalibrationCommandKind::Reset},
};

/// Заголовок действия: простой межсайтовый POST его не выставит, поэтому
/// браузер обязан пройти preflight, а запрос с чужого origin не пройдёт.
constexpr const char *kCommandHeader = "X-Mount-Command";
constexpr const char *kCommandHeaderValue = "1";
constexpr const char *kBadMediaTypeBody = "{\"error\":\"unsupported_media_type\"}";
constexpr const char *kForbiddenBody = "{\"error\":\"forbidden\"}";

const char *outcomeText(calibration_hal::CommandOutcome outcome)
{
    switch (outcome) {
        case calibration_hal::CommandOutcome::Ok:
            return "ok";
        case calibration_hal::CommandOutcome::NotReady:
            return "not_ready";
        case calibration_hal::CommandOutcome::InvalidState:
            return "invalid_state";
        case calibration_hal::CommandOutcome::InvalidArgument:
            return "invalid_argument";
        case calibration_hal::CommandOutcome::Busy:
            return "busy";
        case calibration_hal::CommandOutcome::StaleSession:
            return "stale_session";
        case calibration_hal::CommandOutcome::Forbidden:
            return "forbidden";
    }
    return "invalid_state";
}

}  // namespace

HttpStatusService::HttpStatusService(state_hal::StateHub &hub,
                                     wifi_hal::WifiApService &access_point,
                                     calibration_hal::CalibrationService &calibration)
    : hub_(hub),
      access_point_(access_point),
      calibration_(calibration),
      port_(*this),
      controller_(port_),
      exit_sem_storage_{},
      delete_gate_storage_{},
      call_mutex_storage_{}
{
    exit_sem_ = xSemaphoreCreateBinaryStatic(&exit_sem_storage_);
    delete_gate_ = xSemaphoreCreateBinaryStatic(&delete_gate_storage_);
    call_mutex_ = xSemaphoreCreateMutexStatic(&call_mutex_storage_);
}

HttpStatusService::~HttpStatusService()
{
    // Пока сервер жив, его обработчики ссылаются на этот объект. Поэтому
    // уничтожение допустимо только после подтверждённого завершения.
    while (stop() != ESP_OK) {
        ESP_LOGE(kTag, "destructor: HTTP shutdown not confirmed, waiting");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t HttpStatusService::start()
{
    if (exit_sem_ == nullptr || delete_gate_ == nullptr ||
        call_mutex_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    // start()/stop() сериализованы: модель вызовов задана явно.
    (void)xSemaphoreTake(call_mutex_, portMAX_DELAY);

    esp_err_t result = ESP_OK;
    if (task_ != nullptr) {
        result = ESP_ERR_INVALID_STATE;
    } else {
        // Второй экземпляр запрещён: владение могло сохраниться после
        // неподтверждённого останова.
        switch (controller_.state()) {
            case web::ServerState::Stopped:
            case web::ServerState::StartFailed:
                break;
            case web::ServerState::Starting:
            case web::ServerState::Running:
            case web::ServerState::Stopping:
            case web::ServerState::StopFailed:
                result = ESP_ERR_INVALID_STATE;
                break;
        }
    }

    if (result == ESP_OK) {
        shutdown_requested_.store(false, std::memory_order_release);
        shutdown_result_.store(ESP_OK, std::memory_order_release);
        retry_gate_.clear();
        shutdown_protocol_.reset();
        while (xSemaphoreTake(exit_sem_, 0) == pdTRUE) {
        }

        const BaseType_t created = xTaskCreatePinnedToCore(
            taskEntry, "web_sup", CONFIG_WEB_SUPERVISOR_STACK, this,
            CONFIG_WEB_SUPERVISOR_PRIORITY, &task_,
            CONFIG_WEB_SUPERVISOR_CORE);
        if (created != pdPASS) {
            task_ = nullptr;
            result = ESP_ERR_NO_MEM;
        }
    }

    (void)xSemaphoreGive(call_mutex_);
    return result;
}

esp_err_t HttpStatusService::stop()
{
    if (call_mutex_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    (void)xSemaphoreTake(call_mutex_, portMAX_DELAY);

    if (task_ == nullptr) {
        // Супервизора нет: результат определяется сохранённым владением.
        const esp_err_t idle =
            controller_.state() == web::ServerState::StopFailed ? ESP_FAIL
                                                                : ESP_OK;
        (void)xSemaphoreGive(call_mutex_);
        return idle;
    }

    // Только первый запрос разрешает уведомление. Повтор после таймаута
    // лишь ожидает подтверждение: уведомлять самоуничтожавшуюся задачу
    // небезопасно, а её handle мог уже освободиться.
    const web::ShutdownProtocol::WaitAction action =
        shutdown_protocol_.beginStop();
    if (action == web::ShutdownProtocol::WaitAction::Completed) {
        const esp_err_t done = shutdown_result_.load(std::memory_order_acquire);
        (void)xSemaphoreGive(call_mutex_);
        return done;
    }
    if (action == web::ShutdownProtocol::WaitAction::NotifyThenWait) {
        shutdown_requested_.store(true, std::memory_order_release);
        (void)xTaskNotifyGive(task_);
    }

    if (xSemaphoreTake(exit_sem_, pdMS_TO_TICKS(CONFIG_WEB_STOP_TIMEOUT_MS)) !=
        pdTRUE) {
        // Владение сохраняется; следующий вызов только продолжит ожидание.
        shutdown_protocol_.onWaitTimeout();
        (void)xSemaphoreGive(call_mutex_);
        return ESP_ERR_TIMEOUT;
    }

    // Подтверждение получено: задача ждёт удаления. Удаляет владелец, чтобы
    // сохранённый handle не стал недействительным до возврата из stop().
    vTaskDelete(task_);
    task_ = nullptr;
    shutdown_protocol_.onCompleted();

    const esp_err_t result = shutdown_result_.load(std::memory_order_acquire);
    (void)xSemaphoreGive(call_mutex_);
    return result;
}

HttpStatusCounters HttpStatusService::counters() const
{
    HttpStatusCounters snapshot;
    snapshot.lifecycle = published_lifecycle_.load(std::memory_order_acquire);
    snapshot.handle_present = handle_present_.load(std::memory_order_acquire);
    snapshot.requests_total = requests_total_.load(std::memory_order_relaxed);
    snapshot.page_requests = page_requests_.load(std::memory_order_relaxed);
    snapshot.state_requests = state_requests_.load(std::memory_order_relaxed);
    snapshot.calibration_requests =
        calibration_requests_.load(std::memory_order_relaxed);
    snapshot.calibration_commands =
        calibration_commands_.load(std::memory_order_relaxed);
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

void HttpStatusService::publishState() noexcept
{
    published_lifecycle_.store(controller_.state(), std::memory_order_release);
    handle_present_.store(handle_ != nullptr, std::memory_order_release);
    const web::ServerLifecycle &lifecycle = controller_.lifecycle();
    start_failures_.store(lifecycle.start_failures, std::memory_order_relaxed);
    stop_unconfirmed_.store(lifecycle.stop_unconfirmed,
                            std::memory_order_relaxed);
}

std::uint32_t HttpStatusService::retryIntervalTicks() const noexcept
{
    const std::uint32_t interval_ms =
        controller_.state() == web::ServerState::StartFailed
            ? CONFIG_WEB_START_RETRY_MS
            : CONFIG_WEB_STOP_RETRY_MS;
    return static_cast<std::uint32_t>(pdMS_TO_TICKS(interval_ms));
}

void HttpStatusService::superviseOnce()
{
    const wifi_hal::AccessPointStatus status = access_point_.status();
    const bool ap_running = status.radio_state == wifi::RadioState::Running;
    const bool ap_stopping = status.operation == wifi::Operation::Stopping;

    const bool retry_allowed = retry_gate_.elapsed(
        static_cast<std::uint32_t>(xTaskGetTickCount()),
        retryIntervalTicks());

    const web::ServerAction action =
        controller_.tick(ap_running, ap_stopping, retry_allowed);
    if (action == web::ServerAction::Start) {
        starts_.fetch_add(1, std::memory_order_relaxed);
    } else if (action == web::ServerAction::Stop) {
        stops_.fetch_add(1, std::memory_order_relaxed);
    }

    // Новый отсчёт начинается по завершении реально выполненной попытки, а не
    // при первом появлении флага: иначе повтор шёл бы каждый тик.
    if (action != web::ServerAction::None) {
        if (controller_.retryPending()) {
            retry_gate_.noteAttempt(
                static_cast<std::uint32_t>(xTaskGetTickCount()));
        } else {
            retry_gate_.clear();
        }
    } else if (!controller_.retryPending()) {
        retry_gate_.clear();
    }

    publishState();
}

void HttpStatusService::run()
{
    for (;;) {
        if (shutdown_requested_.load(std::memory_order_acquire)) {
            break;
        }
        superviseOnce();
        (void)ulTaskNotifyTake(
            pdTRUE, pdMS_TO_TICKS(CONFIG_WEB_SUPERVISOR_PERIOD_MS));
    }

    // Финализация идёт теми же правилами, что и обычный останов. Завершение
    // задачи само по себе не доказывает освобождение HTTP-сервера.
    for (;;) {
        const web::ShutdownOutcome outcome = controller_.shutdownOnce();
        publishState();

        if (outcome == web::ShutdownOutcome::Clean) {
            shutdown_result_.store(ESP_OK, std::memory_order_release);
            break;
        }

        shutdown_result_.store(last_error_.load(std::memory_order_acquire),
                               std::memory_order_release);

        if (outcome == web::ShutdownOutcome::Terminal) {
            // Останов не подтверждён и повтор невозможен: handle сохраняется,
            // объект уничтожать нельзя. Задача остаётся владельцем.
            ESP_LOGE(kTag,
                     "shutdown: server stop not confirmed, ownership retained");
            for (;;) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }

        ESP_LOGW(kTag, "shutdown: retrying server stop");
        vTaskDelay(pdMS_TO_TICKS(CONFIG_WEB_STOP_RETRY_MS));
    }

    // Подтверждение завершения. Задачу удаляет владелец в stop(): удаление
    // самой собой сделало бы сохранённый handle недействительным раньше, чем
    // вызывающий об этом узнает. До удаления задача не обращается к this.
    (void)xSemaphoreGive(exit_sem_);
    (void)xSemaphoreTake(delete_gate_, portMAX_DELAY);
}

web::StartOutcome HttpStatusService::portStart()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = CONFIG_WEB_TASK_STACK;
    config.task_priority = CONFIG_WEB_TASK_PRIORITY;
    config.core_id = CONFIG_WEB_TASK_CORE;
    config.max_open_sockets =
        static_cast<std::uint16_t>(CONFIG_WEB_MAX_OPEN_SOCKETS);
    config.max_uri_handlers = 16;
    config.max_resp_headers = 4;
    config.backlog_conn = static_cast<std::uint16_t>(CONFIG_WEB_BACKLOG);
    config.recv_wait_timeout =
        static_cast<std::uint16_t>(CONFIG_WEB_RECV_TIMEOUT_S);
    config.send_wait_timeout =
        static_cast<std::uint16_t>(CONFIG_WEB_SEND_TIMEOUT_S);
    config.lru_purge_enable = true;
    // Булева Kconfig-опция в выключенном состоянии не определена, поэтому
    // прямое присваивание сломало бы сборку при WEB_TCP_KEEPALIVE=n.
#if CONFIG_WEB_TCP_KEEPALIVE
    config.keep_alive_enable = true;
#else
    config.keep_alive_enable = false;
#endif

    httpd_handle_t handle = nullptr;
    esp_err_t err = httpd_start(&handle, &config);
    if (err != ESP_OK || handle == nullptr) {
        last_error_.store(err == ESP_OK ? ESP_FAIL : err,
                          std::memory_order_release);
        ESP_LOGE(kTag, "httpd start failed: %s", esp_err_to_name(err));
        return web::StartOutcome::Failed;
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
        httpd_uri_t calibration_uri = {};
        calibration_uri.uri = "/api/calibration";
        calibration_uri.method = HTTP_GET;
        calibration_uri.handler = &HttpStatusService::calibrationStatusHandler;
        calibration_uri.user_ctx = this;
        err = httpd_register_uri_handler(handle, &calibration_uri);
    }
    if (err == ESP_OK) {
        for (const PostRoute &route : kCalibrationPosts) {
            httpd_uri_t post_uri = {};
            post_uri.uri = route.uri;
            post_uri.method = HTTP_POST;
            post_uri.handler = &HttpStatusService::calibrationCommandHandler;
            post_uri.user_ctx = this;
            err = httpd_register_uri_handler(handle, &post_uri);
            if (err != ESP_OK) {
                break;
            }
        }
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
            return web::StartOutcome::Failed;
        }
        // Завершение отката не подтверждено: владение сохраняется.
        handle_ = handle;
        if (stop_err == ESP_FAIL) {
            return web::StartOutcome::FailedServerLiveRetryable;
        }
        ESP_LOGE(kTag, "start rollback stop failed: %s",
                 esp_err_to_name(stop_err));
        return web::StartOutcome::FailedServerLiveTerminal;
    }

    handle_ = handle;
    last_error_.store(ESP_OK, std::memory_order_release);
    ESP_LOGI(kTag, "http status page started: port=%u sockets=%u stack=%u",
             static_cast<unsigned>(config.server_port),
             static_cast<unsigned>(config.max_open_sockets),
             static_cast<unsigned>(config.stack_size));
    return web::StartOutcome::Started;
}

web::StopOutcome HttpStatusService::portStop()
{
    if (handle_ == nullptr) {
        return web::StopOutcome::Confirmed;
    }

    const esp_err_t err = httpd_stop(handle_);
    if (err == ESP_OK) {
        handle_ = nullptr;
        last_error_.store(ESP_OK, std::memory_order_release);
        ESP_LOGI(kTag, "http status page stopped");
        return web::StopOutcome::Confirmed;
    }

    last_error_.store(err, std::memory_order_release);
    if (err == ESP_FAIL) {
        // По контракту сервер жив и handle валиден: повтор допустим.
        ESP_LOGW(kTag, "httpd stop unconfirmed, retry allowed");
        return web::StopOutcome::NotConfirmedRetryable;
    }
    ESP_LOGE(kTag, "httpd stop failed: %s", esp_err_to_name(err));
    return web::StopOutcome::NotConfirmedTerminal;
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

esp_err_t HttpStatusService::sendCalibrationStatus(httpd_req_t *req)
{
    const calibration_hal::CalibrationStatus status = calibration_.status();
    char buffer[web::kMaxCalibrationJsonBytes];
    std::size_t written = 0;
    if (web::calibrationStatusJson(toView(status), buffer, sizeof(buffer), &written) !=
        web::CalibrationJsonResult::Ok) {
        serialize_overflows_.fetch_add(1, std::memory_order_relaxed);
        request_failures_.fetch_add(1, std::memory_order_relaxed);
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, kJsonContentType);
        httpd_resp_set_hdr(req, kCacheControl, kNoStore);
        return httpd_resp_send(req, kOverflowBody, HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_set_status(req, "200 OK");
    httpd_resp_set_type(req, kJsonContentType);
    httpd_resp_set_hdr(req, kCacheControl, kNoStore);
    const esp_err_t err = httpd_resp_send(req, buffer, static_cast<ssize_t>(written));
    if (err != ESP_OK) {
        request_failures_.fetch_add(1, std::memory_order_relaxed);
    }
    return err;
}

esp_err_t HttpStatusService::calibrationStatusHandler(httpd_req_t *req)
{
    auto *self = static_cast<HttpStatusService *>(req->user_ctx);
    if (self == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    self->requests_total_.fetch_add(1, std::memory_order_relaxed);
    self->calibration_requests_.fetch_add(1, std::memory_order_relaxed);
    return self->sendCalibrationStatus(req);
}

esp_err_t HttpStatusService::calibrationCommandHandler(httpd_req_t *req)
{
    auto *self = static_cast<HttpStatusService *>(req->user_ctx);
    if (self == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    self->requests_total_.fetch_add(1, std::memory_order_relaxed);
    self->calibration_commands_.fetch_add(1, std::memory_order_relaxed);

    web::CalibrationCommandKind kind = web::CalibrationCommandKind::Stop;
    bool known = false;
    for (const PostRoute &route : kCalibrationPosts) {
        if (std::strcmp(req->uri, route.uri) == 0) {
            kind = route.kind;
            known = true;
            break;
        }
    }
    if (!known) {
        httpd_resp_set_status(req, "404 Not Found");
        httpd_resp_set_type(req, kJsonContentType);
        return httpd_resp_send(req, kNotFoundBody, HTTPD_RESP_USE_STRLEN);
    }
    // Разграничение наблюдателя и управляющего клиента: изменяющий запрос
    // обязан нести заголовок действия и JSON-тип. Простой межсайтовый POST
    // такой заголовок не выставит, поэтому preflight обязателен.
    char action_header[16];
    if (httpd_req_get_hdr_value_str(req, kCommandHeader, action_header,
                                    sizeof(action_header)) != ESP_OK ||
        std::strcmp(action_header, kCommandHeaderValue) != 0) {
        self->request_failures_.fetch_add(1, std::memory_order_relaxed);
        httpd_resp_set_status(req, "403 Forbidden");
        httpd_resp_set_type(req, kJsonContentType);
        return httpd_resp_send(req, kForbiddenBody, HTTPD_RESP_USE_STRLEN);
    }
    char content_type[40];
    if (httpd_req_get_hdr_value_str(req, "Content-Type", content_type,
                                    sizeof(content_type)) != ESP_OK ||
        std::strncmp(content_type, "application/json", 16) != 0) {
        httpd_resp_set_status(req, "415 Unsupported Media Type");
        httpd_resp_set_type(req, kJsonContentType);
        return httpd_resp_send(req, kBadMediaTypeBody, HTTPD_RESP_USE_STRLEN);
    }
    if (req->content_len < 0 ||
        static_cast<std::size_t>(req->content_len) > kMaxCommandBodyBytes) {
        httpd_resp_set_status(req, "413 Payload Too Large");
        httpd_resp_set_type(req, kJsonContentType);
        return httpd_resp_send(req, kTooLargeBody, HTTPD_RESP_USE_STRLEN);
    }

    char body[kMaxCommandBodyBytes + 1];
    std::size_t received = 0;
    const std::size_t total = static_cast<std::size_t>(req->content_len);
    while (received < total) {
        const int ret = httpd_req_recv(req, body + received, total - received);
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (ret <= 0) {
            self->request_failures_.fetch_add(1, std::memory_order_relaxed);
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_set_type(req, kJsonContentType);
            return httpd_resp_send(req, kBadRequestBody, HTTPD_RESP_USE_STRLEN);
        }
        received += static_cast<std::size_t>(ret);
    }
    body[received] = '\0';

    web::CalibrationCommand command{};
    if (web::parseCalibrationCommand(kind, body, received, command) !=
        web::CalibrationCommandResult::Ok) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, kJsonContentType);
        return httpd_resp_send(req, kBadRequestBody, HTTPD_RESP_USE_STRLEN);
    }

    calibration_hal::CommandOutcome outcome = calibration_hal::CommandOutcome::InvalidState;
    std::uint32_t created_session = 0;
    std::uint32_t created_token = 0;
    switch (kind) {
        case web::CalibrationCommandKind::CreateSession: {
            calibration_hal::SessionRequest request{};
            request.axis = command.axis == 1 ? motion::AxisId::Altitude
                                             : motion::AxisId::Azimuth;
            request.direction = command.direction == 1 ? motion::Direction::Backward
                                                       : motion::Direction::Forward;
            request.revolutions = command.revolutions;
            request.repeats = command.repeats;
            outcome = self->calibration_.createSession(request, created_session,
                                                       created_token);
            break;
        }
        case web::CalibrationCommandKind::Move:
            outcome = self->calibration_.move(
                command.session_id, command.token,
                command.direction == 1 ? motion::Direction::Backward
                                       : motion::Direction::Forward,
                command.rate, command.motion_generation);
            break;
        case web::CalibrationCommandKind::Stop:
            outcome = self->calibration_.stop(command.session_id, command.token);
            break;
        case web::CalibrationCommandKind::ConfirmStart:
            outcome = self->calibration_.confirmStart(command.session_id, command.token);
            break;
        case web::CalibrationCommandKind::ConfirmEnd:
            outcome = self->calibration_.confirmEnd(command.session_id, command.token);
            break;
        case web::CalibrationCommandKind::Resume:
            outcome = self->calibration_.resume(command.session_id, command.token);
            break;
        case web::CalibrationCommandKind::Abandon:
            outcome = self->calibration_.abandon(command.session_id, command.token);
            break;
        case web::CalibrationCommandKind::DeleteMeasurement:
            outcome = self->calibration_.deleteMeasurement(command.session_id, command.token,
                                                           command.index);
            break;
        case web::CalibrationCommandKind::AddRepeat:
            outcome = self->calibration_.addRepeat(command.session_id, command.token);
            break;
        case web::CalibrationCommandKind::Apply:
            outcome = self->calibration_.apply(command.session_id, command.token,
                                               command.acknowledge_low_repeatability);
            break;
        case web::CalibrationCommandKind::Close:
            outcome = self->calibration_.closeSession(command.session_id, command.token);
            break;
        case web::CalibrationCommandKind::Reset:
            outcome = self->calibration_.resetAxis(
                command.axis == 1 ? motion::AxisId::Altitude : motion::AxisId::Azimuth,
                command.confirm);
            break;
    }

    if (outcome == calibration_hal::CommandOutcome::Ok) {
        if (kind == web::CalibrationCommandKind::CreateSession) {
            // Токен выдаётся только создателю и не публикуется в статусе.
            char created[96];
            std::snprintf(created, sizeof(created),
                          "{\"session_id\":%u,\"token\":%u}",
                          static_cast<unsigned>(created_session),
                          static_cast<unsigned>(created_token));
            httpd_resp_set_status(req, "200 OK");
            httpd_resp_set_type(req, kJsonContentType);
            httpd_resp_set_hdr(req, kCacheControl, kNoStore);
            return httpd_resp_send(req, created, HTTPD_RESP_USE_STRLEN);
        }
        return self->sendCalibrationStatus(req);
    }
    char error_body[64];
    std::snprintf(error_body, sizeof(error_body), "{\"error\":\"%s\"}",
                  outcomeText(outcome));
    const char *status = "409 Conflict";
    if (outcome == calibration_hal::CommandOutcome::InvalidArgument) {
        status = "400 Bad Request";
    } else if (outcome == calibration_hal::CommandOutcome::Busy) {
        status = "429 Too Many Requests";
    } else if (outcome == calibration_hal::CommandOutcome::Forbidden) {
        status = "403 Forbidden";
    }
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, kJsonContentType);
    httpd_resp_set_hdr(req, kCacheControl, kNoStore);
    return httpd_resp_send(req, error_body, HTTPD_RESP_USE_STRLEN);
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
