/**
 * @file wifi_ap_service.cpp
 * @brief Служба точки доступа и протокол завершения задачи.
 *
 * Владение и порядок:
 *  - lifecycle_ изменяют только start() и stop() под api_mutex_;
 *  - задача не берёт api_mutex_ никогда, поэтому stop() может ждать её;
 *  - teardown_result_ пишется задачей без мьютекса, затем публикуется
 *    task_exited_ (release) и выдаётся exit_sem_;
 *  - окончательным подтверждением служит полученный exit_sem_;
 *  - признак выполненной финализации — task_ == nullptr.
 */

#include "wifi_hal/wifi_ap_service.hpp"

#include <cstdint>
#include <cstring>

#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "sdkconfig.h"
#include "wifi_hal/settings_store.hpp"

// 0004: сохранение RF-калибровки должно быть выключено, иначе переключение
// точки доступа может писать в flash. Значение задаётся sdkconfig.defaults.
#ifdef CONFIG_ESP_PHY_CALIBRATION_AND_DATA_STORAGE
#error "CONFIG_ESP_PHY_CALIBRATION_AND_DATA_STORAGE must be disabled: see sdkconfig.defaults"
#endif

namespace wifi_hal {
namespace {

constexpr const char *kTag = "wifi";

constexpr TickType_t kSupervisionSliceMs = 100;

void logField(const char *name, const FieldStatus &status)
{
    if (status.issue == SettingIssue::InvalidValue) {
        ESP_LOGW(kTag, "settings %s: nvs value ignored", name);
        return;
    }
    if (status.issue == SettingIssue::ReadError) {
        ESP_LOGW(kTag, "settings %s: nvs read error %s", name,
                 esp_err_to_name(status.error));
        return;
    }
    ESP_LOGI(kTag, "settings %s: source=%s", name,
             status.source == SettingSource::Nvs ? "nvs" : "kconfig");
}

void logLoadResult(const LoadResult &load)
{
    logField("ssid", load.ssid);
    // Значение пароля не логируется никогда.
    logField("password", load.password);
    logField("channel", load.channel);
    logField("max_stations", load.max_stations);
    logField("hidden", load.hidden);
    logField("start_on_boot", load.start_on_boot);
    if (load.any_invalid) {
        ESP_LOGW(kTag, "settings: invalid nvs values were replaced by kconfig");
    }
    if (load.any_read_error) {
        ESP_LOGE(kTag, "settings: nvs read errors, status=%s",
                 esp_err_to_name(load.status));
    }
}

}  // namespace

class WifiApService::ApiLock {
public:
    explicit ApiLock(SemaphoreHandle_t mutex) : mutex_(mutex)
    {
        if (mutex_ != nullptr) {
            (void)xSemaphoreTake(mutex_, portMAX_DELAY);
        }
    }

    ~ApiLock()
    {
        if (mutex_ != nullptr) {
            (void)xSemaphoreGive(mutex_);
        }
    }

    ApiLock(const ApiLock &) = delete;
    ApiLock &operator=(const ApiLock &) = delete;

private:
    SemaphoreHandle_t mutex_ = nullptr;
};

class WifiApService::StateLock {
public:
    explicit StateLock(SemaphoreHandle_t mutex) : mutex_(mutex)
    {
        if (mutex_ != nullptr) {
            (void)xSemaphoreTake(mutex_, portMAX_DELAY);
        }
    }

    ~StateLock()
    {
        if (mutex_ != nullptr) {
            (void)xSemaphoreGive(mutex_);
        }
    }

    StateLock(const StateLock &) = delete;
    StateLock &operator=(const StateLock &) = delete;

private:
    SemaphoreHandle_t mutex_ = nullptr;
};

WifiApService::WifiApService()
{
    api_mutex_ = xSemaphoreCreateMutexStatic(&api_mutex_storage_);
    state_mutex_ = xSemaphoreCreateMutexStatic(&state_mutex_storage_);
    exit_sem_ = xSemaphoreCreateBinaryStatic(&exit_sem_storage_);
}

WifiApService::~WifiApService()
{
    while (stop() == ESP_ERR_TIMEOUT) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t WifiApService::start()
{
    if (api_mutex_ == nullptr || state_mutex_ == nullptr ||
        exit_sem_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    ApiLock lock(api_mutex_);
    if (lifecycle_ != wifi::Lifecycle::Stopped || task_ != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    // Сброс состояния предыдущего запуска. После успешной финализации
    // подтверждение уже потреблено: очистка семафора — защитная мера.
    shutdown_requested_.store(false, std::memory_order_release);
    resync_requested_.store(false, std::memory_order_release);
    teardown_result_ = ESP_OK;
    task_exited_.store(false, std::memory_order_release);
    while (xSemaphoreTake(exit_sem_, 0) == pdTRUE) {
    }
    (void)admission_.dropPending(wifi::ToggleAdmission::DropReason::Shutdown);

    lifecycle_ = wifi::Lifecycle::Starting;
    published_lifecycle_.store(wifi::Lifecycle::Starting);

    esp_err_t err = SettingsStore::initNvs();
    if (err != ESP_OK) {
        return failStart(err);
    }

    SettingsStore store;
    const LoadResult load = store.load();
    logLoadResult(load);
    if (load.status != ESP_OK) {
        return failStart(load.status);
    }

    const wifi::ConfigError config_error = wifi::validate(load.settings);
    if (config_error != wifi::ConfigError::None) {
        ESP_LOGE(kTag, "invalid access point configuration: %s",
                 wifi::errorName(config_error));
        return failStart(ESP_ERR_INVALID_ARG);
    }

    err = initialize(load.settings);
    if (err != ESP_OK) {
        return failStart(err);
    }

    err = initialStart(load.settings.start_on_boot);
    if (err != ESP_OK) {
        return failStart(err);
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        taskEntry, "wifi_ap", CONFIG_WIFI_TASK_STACK, this,
        CONFIG_WIFI_TASK_PRIORITY, &task_, CONFIG_WIFI_TASK_CORE);
    if (created != pdPASS) {
        task_ = nullptr;
        return failStart(ESP_ERR_NO_MEM);
    }

    lifecycle_ = wifi::Lifecycle::Running;
    published_lifecycle_.store(wifi::Lifecycle::Running);
    ESP_LOGI(kTag, "access point service started");
    return ESP_OK;
}

esp_err_t WifiApService::stop()
{
    if (api_mutex_ == nullptr) {
        return ESP_OK;
    }

    // RAII: мьютекс освобождается на любом выходе, включая таймаут.
    ApiLock lock(api_mutex_);

    if (lifecycle_ == wifi::Lifecycle::Stopped) {
        return ESP_OK;
    }
    if (lifecycle_ == wifi::Lifecycle::Stopping && task_ == nullptr) {
        // Финализация уже выполнена: код сохранён, ожидания нет.
        return teardown_result_;
    }

    if (lifecycle_ != wifi::Lifecycle::Stopping) {
        lifecycle_ = wifi::Lifecycle::Stopping;
        published_lifecycle_.store(wifi::Lifecycle::Stopping);
        (void)admission_.dropPending(wifi::ToggleAdmission::DropReason::Shutdown);
        shutdown_requested_.store(true, std::memory_order_release);
        if (task_ != nullptr) {
            (void)xTaskNotifyGive(task_);
        }
    }

    // Первая финализация запуска всегда ждёт подтверждение, даже если
    // task_exited_ уже наблюдается как true.
    const TickType_t timeout = pdMS_TO_TICKS(CONFIG_WIFI_STOP_TIMEOUT_MS);
    if (xSemaphoreTake(exit_sem_, timeout) != pdTRUE) {
        ESP_LOGE(kTag, "task stop confirmation timeout");
        return ESP_ERR_TIMEOUT;
    }
    return finalizeStopLocked();
}

esp_err_t WifiApService::requestToggle()
{
    if (api_mutex_ == nullptr || state_mutex_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }
    if (xSemaphoreTake(api_mutex_, 0) != pdTRUE) {
        rejected_api_busy_.fetch_add(1, std::memory_order_relaxed);
        return ESP_ERR_TIMEOUT;
    }
    if (xSemaphoreTake(state_mutex_, 0) != pdTRUE) {
        rejected_state_busy_.fetch_add(1, std::memory_order_relaxed);
        (void)xSemaphoreGive(api_mutex_);
        return ESP_ERR_TIMEOUT;
    }

    const wifi::RadioState radio = coordinator_.radioState();
    const wifi::ToggleAdmission::Outcome outcome =
        admission_.tryAdmit(lifecycle_, radio);
    if (outcome.result == wifi::ToggleAdmission::Result::Unconfirmed) {
        // Отказ в переключении, но повод для ресинхронизации доставляется.
        resync_requested_.store(true, std::memory_order_release);
    }
    (void)xSemaphoreGive(state_mutex_);

    // api_mutex_ ещё удерживается: stop() не может обнулить task_ между
    // проверкой и уведомлением.
    if (outcome.wake_task && task_ != nullptr) {
        (void)xTaskNotifyGive(task_);
    }
    (void)xSemaphoreGive(api_mutex_);

    switch (outcome.result) {
        case wifi::ToggleAdmission::Result::Accepted:
            return ESP_OK;
        case wifi::ToggleAdmission::Result::Full:
            return ESP_ERR_NO_MEM;
        case wifi::ToggleAdmission::Result::Unconfirmed:
        case wifi::ToggleAdmission::Result::NotRunning:
            return ESP_ERR_INVALID_STATE;
    }
    return ESP_ERR_INVALID_STATE;
}

AccessPointStatus WifiApService::status() const
{
    AccessPointStatus snapshot;
    snapshot.lifecycle = published_lifecycle_.load(std::memory_order_acquire);
    snapshot.task_exited = task_exited_.load(std::memory_order_acquire);

    if (state_mutex_ != nullptr) {
        (void)xSemaphoreTake(state_mutex_, portMAX_DELAY);
        snapshot.radio_state = coordinator_.radioState();
        snapshot.operation = coordinator_.operation();
        snapshot.stations = coordinator_.stations();
        snapshot.configured_start_on_boot = configured_start_on_boot_;
        for (std::size_t i = 0; i < wifi::kSsidCapacity; ++i) {
            snapshot.ssid[i] = ssid_[i];
        }
        snapshot.channel = channel_;
        snapshot.auth_mode = auth_mode_;
        snapshot.event_timeouts = coordinator_.event_timeouts;
        snapshot.operation_errors = coordinator_.operation_errors;
        snapshot.resyncs = coordinator_.resyncs;
        snapshot.starts = coordinator_.starts;
        snapshot.stops = coordinator_.stops;
        snapshot.last_error = last_error_;
        (void)xSemaphoreGive(state_mutex_);
    }

    // Атомарные счётчики: снимок не транзакционный по разным группам полей.
    snapshot.pending_toggles = admission_.pending();
    snapshot.accepted_toggles = admission_.accepted.load();
    snapshot.rejected_not_running = admission_.rejected_not_running.load();
    snapshot.rejected_unconfirmed = admission_.rejected_unconfirmed.load();
    snapshot.rejected_full = admission_.rejected_full.load();
    snapshot.executed_toggles = admission_.executed.load();
    snapshot.failed_on_shutdown = admission_.failed_on_shutdown.load();
    snapshot.failed_on_timeout = admission_.failed_on_timeout.load();
    snapshot.failed_on_error = admission_.failed_on_error.load();
    snapshot.dropped_on_shutdown = admission_.dropped_on_shutdown.load();
    snapshot.dropped_on_timeout = admission_.dropped_on_timeout.load();
    snapshot.dropped_on_error = admission_.dropped_on_error.load();
    snapshot.rejected_api_busy = rejected_api_busy_.load();
    snapshot.rejected_state_busy = rejected_state_busy_.load();
    snapshot.internal_errors = internal_errors_.load();
    snapshot.teardown_result = teardown_result_;
    return snapshot;
}

void WifiApService::taskEntry(void *arg)
{
    static_cast<WifiApService *>(arg)->run();
}

void WifiApService::run()
{
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        for (;;) {
            if (shutdown_requested_.load(std::memory_order_acquire)) {
                break;
            }
            if (resync_requested_.exchange(false, std::memory_order_acq_rel)) {
                doResyncOnce();
                continue;
            }
            if (!admission_.takeOne()) {
                break;
            }
            doToggleOnce();
        }

        if (!shutdown_requested_.load(std::memory_order_acquire)) {
            continue;
        }

        (void)admission_.dropPending(
            wifi::ToggleAdmission::DropReason::Shutdown);
        const esp_err_t result = teardown();
        teardown_result_ = result;
        {
            StateLock lock(state_mutex_);
            if (result == ESP_OK) {
                coordinator_.finishShutdown();
            } else {
                last_error_ = result;
            }
        }
        // После публикации задача обращается только к собственному стеку.
        task_exited_.store(true, std::memory_order_release);
        (void)xSemaphoreGive(exit_sem_);
        vTaskDelete(nullptr);
        return;
    }
}

void WifiApService::eventHandler(void *arg, esp_event_base_t base, int32_t id,
                                 void *data)
{
    (void)base;
    (void)data;
    static_cast<WifiApService *>(arg)->handleWifiEvent(id);
}

void WifiApService::handleWifiEvent(int32_t id)
{
    wifi::Operation operation = wifi::Operation::None;
    wifi::RadioState radio = wifi::RadioState::Unknown;
    std::uint8_t stations = 0;
    {
        StateLock lock(state_mutex_);
        switch (id) {
            case WIFI_EVENT_AP_START:
                coordinator_.onApStarted();
                break;
            case WIFI_EVENT_AP_STOP:
                coordinator_.onApStopped();
                break;
            case WIFI_EVENT_AP_STACONNECTED:
                coordinator_.onStaConnected();
                break;
            case WIFI_EVENT_AP_STADISCONNECTED:
                coordinator_.onStaDisconnected();
                break;
            default:
                break;
        }
        operation = coordinator_.operation();
        radio = coordinator_.radioState();
        stations = coordinator_.stations();
    }

    // Логирование только после отпускания мьютекса состояния.
    switch (id) {
        case WIFI_EVENT_AP_START:
            ESP_LOGI(kTag, "AP started: radio=%u operation=%u",
                     static_cast<unsigned>(radio),
                     static_cast<unsigned>(operation));
            break;
        case WIFI_EVENT_AP_STOP:
            ESP_LOGI(kTag, "AP stopped: radio=%u operation=%u",
                     static_cast<unsigned>(radio),
                     static_cast<unsigned>(operation));
            break;
        case WIFI_EVENT_AP_STACONNECTED:
            ESP_LOGI(kTag, "station connected: stations=%u",
                     static_cast<unsigned>(stations));
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(kTag, "station disconnected: stations=%u",
                     static_cast<unsigned>(stations));
            break;
        default:
            break;
    }
}

esp_err_t WifiApService::initNetwork()
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(kTag, "esp_netif_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(kTag, "event loop create failed: %s", esp_err_to_name(err));
        return err;
    }

    netif_ = esp_netif_create_default_wifi_ap();
    if (netif_ == nullptr) {
        ESP_LOGE(kTag, "AP netif creation failed");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t WifiApService::initDriver()
{
    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return err;
    }
    wifi_initialized_ = true;

    // Конфигурация AP хранится только в RAM: драйвер не пишет её в NVS.
    err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "esp_wifi_set_storage failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "esp_wifi_set_mode failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t WifiApService::applyConfig(
    const wifi::AccessPointSettings &settings)
{
    wifi_config_t config{};

    const std::size_t ssid_length = std::strlen(settings.ap.ssid);
    if (ssid_length == 0 || ssid_length > sizeof(config.ap.ssid)) {
        return ESP_ERR_INVALID_ARG;
    }
    std::memcpy(config.ap.ssid, settings.ap.ssid, ssid_length);
    // SSID ровно из 32 символов заполняет всё поле драйвера: NUL не нужен.
    config.ap.ssid_len = static_cast<std::uint8_t>(ssid_length);

    (void)std::strncpy(reinterpret_cast<char *>(config.ap.password),
                       settings.ap.password, sizeof(config.ap.password) - 1);
    const bool open_network = settings.ap.password[0] == '\0';
    config.ap.channel = settings.ap.channel;
    config.ap.authmode = open_network ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;
    config.ap.ssid_hidden = settings.ap.hidden ? 1 : 0;
    config.ap.max_connection = settings.ap.max_stations;
    config.ap.pmf_cfg.required = false;
    config.ap.pmf_cfg.capable = false;

    const esp_err_t err = esp_wifi_set_config(WIFI_IF_AP, &config);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "esp_wifi_set_config failed: %s", esp_err_to_name(err));
        return err;
    }

    {
        StateLock lock(state_mutex_);
        for (std::size_t i = 0; i < wifi::kSsidCapacity; ++i) {
            ssid_[i] = settings.ap.ssid[i];
        }
        channel_ = settings.ap.channel;
        auth_mode_ = static_cast<std::uint8_t>(open_network ? WIFI_AUTH_OPEN
                                                           : WIFI_AUTH_WPA2_PSK);
    }
    return ESP_OK;
}

esp_err_t WifiApService::registerHandler()
{
    const esp_err_t err = esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiApService::eventHandler, this,
        &handler_instance_);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "event handler register failed: %s",
                 esp_err_to_name(err));
        return err;
    }
    handler_registered_ = true;
    return ESP_OK;
}

esp_err_t WifiApService::initialize(
    const wifi::AccessPointSettings &settings)
{
    esp_err_t err = initNetwork();
    if (err != ESP_OK) {
        return err;
    }
    err = initDriver();
    if (err != ESP_OK) {
        return err;
    }
    err = applyConfig(settings);
    if (err != ESP_OK) {
        return err;
    }
    return registerHandler();
}

esp_err_t WifiApService::initialStart(bool start_on_boot)
{
    {
        StateLock lock(state_mutex_);
        coordinator_.beginStartup();
        configured_start_on_boot_ = start_on_boot;
    }

    if (!start_on_boot) {
        {
            StateLock lock(state_mutex_);
            coordinator_.startupSucceeded(false);
        }
        ESP_LOGW(kTag,
                 "access point not started: start_on_boot disabled by "
                 "configuration");
        return ESP_OK;
    }

    const esp_err_t err = esp_wifi_start();
    if (err != ESP_OK) {
        {
            StateLock lock(state_mutex_);
            coordinator_.onOperationFailed();
            last_error_ = err;
        }
        ESP_LOGE(kTag, "esp_wifi_start failed: %s", esp_err_to_name(err));
        return err;
    }

    if (supervise(wifi::RadioState::Running) == Supervision::Confirmed) {
        {
            StateLock lock(state_mutex_);
            coordinator_.startupSucceeded(true);
            last_error_ = ESP_OK;
        }
        return ESP_OK;
    }

    // Одна попытка ресинхронизации. Подтвердить запуск она не может:
    // ResyncStop завершается только остановкой.
    ESP_LOGW(kTag, "AP_START not confirmed: single resync attempt");
    doResyncOnce();
    {
        StateLock lock(state_mutex_);
        coordinator_.startupFailed();
        last_error_ = ESP_ERR_TIMEOUT;
    }
    ESP_LOGE(kTag, "access point startup not confirmed in time");
    return ESP_ERR_TIMEOUT;
}

esp_err_t WifiApService::failStart(esp_err_t cause)
{
    const esp_err_t teardown_error = teardown();
    if (teardown_error != ESP_OK) {
        lifecycle_ = wifi::Lifecycle::Stopping;
        published_lifecycle_.store(wifi::Lifecycle::Stopping);
        ESP_LOGE(kTag, "start failed (%s), rollback failed (%s): reboot needed",
                 esp_err_to_name(cause), esp_err_to_name(teardown_error));
        return teardown_error;
    }
    lifecycle_ = wifi::Lifecycle::Stopped;
    published_lifecycle_.store(wifi::Lifecycle::Stopped);
    ESP_LOGE(kTag, "start failed: %s", esp_err_to_name(cause));
    return cause;
}

esp_err_t WifiApService::teardown()
{
    esp_err_t first_error = ESP_OK;

    // 1. Сначала прекращаются callback: без этого нельзя освобождать
    //    ресурсы, к которым они обращаются.
    if (handler_registered_ && handler_instance_ != nullptr) {
        const esp_err_t err = esp_event_handler_instance_unregister(
            WIFI_EVENT, ESP_EVENT_ANY_ID, handler_instance_);
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "unregister failed: %s: resources retained",
                     esp_err_to_name(err));
            return err;
        }
        handler_instance_ = nullptr;
        handler_registered_ = false;
    }

    // 2. Драйвер. Пока он может использовать netif, netif не уничтожается.
    if (wifi_initialized_) {
        const esp_err_t stop_err = esp_wifi_stop();
        if (stop_err != ESP_OK && stop_err != ESP_ERR_WIFI_NOT_STARTED) {
            ESP_LOGE(kTag, "esp_wifi_stop failed: %s: netif retained",
                     esp_err_to_name(stop_err));
            return stop_err;
        }
        const esp_err_t deinit_err = esp_wifi_deinit();
        if (deinit_err != ESP_OK) {
            ESP_LOGE(kTag, "esp_wifi_deinit failed: %s: netif retained",
                     esp_err_to_name(deinit_err));
            return deinit_err;
        }
        wifi_initialized_ = false;
    }

    // 3. Собственный AP netif. ESP-NETIF целиком не деинициализируется.
    if (netif_ != nullptr) {
        esp_netif_destroy_default_wifi(netif_);
        netif_ = nullptr;
    }
    return first_error;
}

esp_err_t WifiApService::finalizeStopLocked()
{
    // Вызывается только под api_mutex_ и никогда его не освобождает.
    const esp_err_t result = teardown_result_;
    task_ = nullptr;
    if (result == ESP_OK) {
        lifecycle_ = wifi::Lifecycle::Stopped;
        published_lifecycle_.store(wifi::Lifecycle::Stopped);
    }
    return result;
}

WifiApService::Supervision WifiApService::supervise(wifi::RadioState expected)
{
    const TickType_t deadline =
        xTaskGetTickCount() + pdMS_TO_TICKS(CONFIG_WIFI_AP_EVENT_TIMEOUT_MS);

    for (;;) {
        if (shutdown_requested_.load(std::memory_order_acquire)) {
            return Supervision::Aborted;
        }

        wifi::Operation operation = wifi::Operation::None;
        wifi::RadioState radio = wifi::RadioState::Unknown;
        {
            StateLock lock(state_mutex_);
            operation = coordinator_.operation();
            radio = coordinator_.radioState();
        }

        if (operation == wifi::Operation::None) {
            return radio == expected ? Supervision::Confirmed
                                     : Supervision::Mismatch;
        }

        const TickType_t now = xTaskGetTickCount();
        if (static_cast<std::int32_t>(now - deadline) >= 0) {
            StateLock lock(state_mutex_);
            coordinator_.onOperationTimeout();
            last_error_ = ESP_ERR_TIMEOUT;
            return Supervision::TimedOut;
        }
        vTaskDelay(pdMS_TO_TICKS(kSupervisionSliceMs));
    }
}

void WifiApService::doToggleOnce()
{
    wifi::Action action = wifi::Action::None;
    {
        StateLock lock(state_mutex_);
        action = coordinator_.beginToggleOperation();
    }

    if (action == wifi::Action::None) {
        internal_errors_.fetch_add(1, std::memory_order_relaxed);
        ESP_LOGE(kTag, "toggle intent not executed: coordinator rejected it");
        admission_.accountFailed(wifi::ToggleAdmission::DropReason::Error);
        (void)admission_.dropPending(wifi::ToggleAdmission::DropReason::Error);
        return;
    }

    const bool starting = action == wifi::Action::StartRadio;
    const wifi::RadioState expected = starting ? wifi::RadioState::Running
                                               : wifi::RadioState::Stopped;
    const esp_err_t err = starting ? esp_wifi_start() : esp_wifi_stop();

    if (!starting && err == ESP_ERR_WIFI_NOT_STARTED) {
        // Документированный безопасный результат «уже остановлено».
        {
            StateLock lock(state_mutex_);
            coordinator_.onAlreadyStopped();
            last_error_ = err;
        }
        admission_.accountExecuted();
        return;
    }

    if (err != ESP_OK) {
        {
            StateLock lock(state_mutex_);
            coordinator_.onOperationFailed();
            last_error_ = err;
        }
        ESP_LOGE(kTag, "wifi %s failed: %s", starting ? "start" : "stop",
                 esp_err_to_name(err));
        admission_.accountFailed(wifi::ToggleAdmission::DropReason::Error);
        (void)admission_.dropPending(wifi::ToggleAdmission::DropReason::Error);
        return;
    }

    switch (supervise(expected)) {
        case Supervision::Confirmed:
            admission_.accountExecuted();
            return;
        case Supervision::TimedOut:
            admission_.accountFailed(wifi::ToggleAdmission::DropReason::Timeout);
            (void)admission_.dropPending(
                wifi::ToggleAdmission::DropReason::Timeout);
            ESP_LOGW(kTag, "wifi operation not confirmed: single resync attempt");
            doResyncOnce();
            return;
        case Supervision::Mismatch:
            internal_errors_.fetch_add(1, std::memory_order_relaxed);
            ESP_LOGE(kTag, "wifi operation resolved to unexpected state");
            admission_.accountFailed(wifi::ToggleAdmission::DropReason::Error);
            return;
        case Supervision::Aborted:
            break;
    }

    // Shutdown: подтверждение имеет приоритет над отменой.
    bool confirmed = false;
    {
        StateLock lock(state_mutex_);
        confirmed = coordinator_.operation() == wifi::Operation::None &&
                    coordinator_.radioState() == expected;
    }
    if (confirmed) {
        admission_.accountExecuted();
    } else {
        admission_.accountFailed(wifi::ToggleAdmission::DropReason::Shutdown);
    }
}

void WifiApService::doResyncOnce()
{
    wifi::Action action = wifi::Action::None;
    {
        StateLock lock(state_mutex_);
        action = coordinator_.beginResync();
    }
    if (action != wifi::Action::ResyncStop) {
        return;  // операция уже выполняется
    }

    const esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_STARTED) {
        StateLock lock(state_mutex_);
        coordinator_.onAlreadyStopped();
        return;
    }
    if (err != ESP_OK) {
        {
            StateLock lock(state_mutex_);
            coordinator_.onOperationFailed();
            last_error_ = err;
        }
        ESP_LOGE(kTag, "resync stop failed: %s", esp_err_to_name(err));
        return;
    }

    // Таймаут оставляет Unknown; новых попыток без повода не запускается.
    (void)supervise(wifi::RadioState::Stopped);
}

}  // namespace wifi_hal
