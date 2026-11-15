/**
 * @file main.cpp
 * @brief Точка входа прошивки контроллера монтировки телескопа.
 *
 * Этап 2: слой движения (motion + motor_hal). main только инициализирует
 * плату, собирает конфигурацию осей, запускает службу движения и, при
 * явно включённом самотесте, выполняет проверочный прогон на стенде.
 */

#include <atomic>
#include <cstdint>

#include "board/board.hpp"
#include "control/joy_mapping.hpp"
#include "control_hal/control_service.hpp"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "input/button_debouncer.hpp"
#include "input_hal/joystick_service.hpp"
#include "motion/axis_config.hpp"
#include "motor_hal/motor_service.hpp"
#include "sdkconfig.h"
#include "state/channels.hpp"
#include "state/observation.hpp"
#include "state_hal/state_hub.hpp"
#include "wifi_hal/wifi_ap_service.hpp"

namespace {

constexpr const char *kTag = "telescope_mount";

/// Полушагов на оборот выходного вала 28BYJ-48 (номинал).
constexpr std::int64_t kHalfStepsPerRevolution = 4096;

/**
 * @brief Считает события кнопки и передаёт двойное нажатие службе WiFi.
 *
 * Вызывается из задачи опроса джойстика (период 20 мс), поэтому здесь нет
 * ни логирования, ни ожиданий: отказ requestToggle() только считается.
 */
class ButtonStatsListener final : public input::ButtonListener {
public:
    struct Stats {
        std::uint32_t pressed = 0;
        std::uint32_t released = 0;
        std::uint32_t click = 0;
        std::uint32_t long_press = 0;
        std::uint32_t double_click = 0;
        std::uint32_t toggle_rejected = 0;
    };

    explicit ButtonStatsListener(wifi_hal::WifiApService &access_point)
        : access_point_(access_point)
    {
    }

    void onButtonEvent(input::ButtonEvent event) override
    {
        switch (event) {
            case input::ButtonEvent::Pressed:
                pressed_.fetch_add(1, std::memory_order_relaxed);
                break;
            case input::ButtonEvent::Released:
                released_.fetch_add(1, std::memory_order_relaxed);
                break;
            case input::ButtonEvent::Click:
                click_.fetch_add(1, std::memory_order_relaxed);
                break;
            case input::ButtonEvent::LongPress:
                long_press_.fetch_add(1, std::memory_order_relaxed);
                break;
            case input::ButtonEvent::DoubleClick:
                double_click_.fetch_add(1, std::memory_order_relaxed);
                if (access_point_.requestToggle() != ESP_OK) {
                    toggle_rejected_.fetch_add(1, std::memory_order_relaxed);
                }
                break;
            case input::ButtonEvent::None:
                break;
        }
    }

    Stats stats() const
    {
        Stats snapshot;
        snapshot.pressed = pressed_.load(std::memory_order_relaxed);
        snapshot.released = released_.load(std::memory_order_relaxed);
        snapshot.click = click_.load(std::memory_order_relaxed);
        snapshot.long_press = long_press_.load(std::memory_order_relaxed);
        snapshot.double_click = double_click_.load(std::memory_order_relaxed);
        snapshot.toggle_rejected =
            toggle_rejected_.load(std::memory_order_relaxed);
        return snapshot;
    }

private:
    wifi_hal::WifiApService &access_point_;
    std::atomic<std::uint32_t> pressed_{0};
    std::atomic<std::uint32_t> released_{0};
    std::atomic<std::uint32_t> click_{0};
    std::atomic<std::uint32_t> long_press_{0};
    std::atomic<std::uint32_t> double_click_{0};
    std::atomic<std::uint32_t> toggle_rejected_{0};
};

void logButtonStats(const ButtonStatsListener::Stats &stats)
{
    ESP_LOGI(kTag,
             "button: pressed=%lu released=%lu click=%lu long_press=%lu "
             "double_click=%lu toggle_rejected=%lu",
             static_cast<unsigned long>(stats.pressed),
             static_cast<unsigned long>(stats.released),
             static_cast<unsigned long>(stats.click),
             static_cast<unsigned long>(stats.long_press),
             static_cast<unsigned long>(stats.double_click),
             static_cast<unsigned long>(stats.toggle_rejected));
}

void logAccessPoint(const wifi_hal::AccessPointStatus &status)
{
    ESP_LOGI(kTag,
             "wifi: lifecycle=%u radio=%u operation=%u ap_running=%d "
             "ssid=%s channel=%u authmode=%u stations=%u pending=%u "
             "start_on_boot=%d toggles=%lu executed=%lu failed_shutdown=%lu "
             "failed_timeout=%lu failed_error=%lu rejected_not_running=%lu "
             "rejected_unconfirmed=%lu rejected_full=%lu dropped_shutdown=%lu "
             "dropped_timeout=%lu dropped_error=%lu event_timeouts=%lu "
             "operation_errors=%lu resyncs=%lu internal_errors=%lu "
             "last_error=%s teardown=%s",
             static_cast<unsigned>(status.lifecycle),
             static_cast<unsigned>(status.radio_state),
             static_cast<unsigned>(status.operation),
             status.radio_state == wifi::RadioState::Running ? 1 : 0,
             status.ssid,
             static_cast<unsigned>(status.channel),
             static_cast<unsigned>(status.auth_mode),
             static_cast<unsigned>(status.stations),
             static_cast<unsigned>(status.pending_toggles),
             status.configured_start_on_boot ? 1 : 0,
             static_cast<unsigned long>(status.accepted_toggles),
             static_cast<unsigned long>(status.executed_toggles),
             static_cast<unsigned long>(status.failed_on_shutdown),
             static_cast<unsigned long>(status.failed_on_timeout),
             static_cast<unsigned long>(status.failed_on_error),
             static_cast<unsigned long>(status.rejected_not_running),
             static_cast<unsigned long>(status.rejected_unconfirmed),
             static_cast<unsigned long>(status.rejected_full),
             static_cast<unsigned long>(status.dropped_on_shutdown),
             static_cast<unsigned long>(status.dropped_on_timeout),
             static_cast<unsigned long>(status.dropped_on_error),
             static_cast<unsigned long>(status.event_timeouts),
             static_cast<unsigned long>(status.operation_errors),
             static_cast<unsigned long>(status.resyncs),
             static_cast<unsigned long>(status.internal_errors),
             esp_err_to_name(status.last_error),
             esp_err_to_name(status.teardown_result));
}

void logChipInfo()
{
    esp_chip_info_t chip_info{};
    esp_chip_info(&chip_info);
    ESP_LOGI(kTag, "chip: %d core(s), silicon revision %d",
             chip_info.cores, chip_info.revision);

    std::uint32_t flash_size_bytes = 0;
    if (esp_flash_get_size(nullptr, &flash_size_bytes) == ESP_OK) {
        ESP_LOGI(kTag, "flash: %u MB",
                 static_cast<unsigned>(flash_size_bytes / (1024U * 1024U)));
    } else {
        ESP_LOGW(kTag, "flash: size unknown");
    }
}

motion::AxisConfig makeAxisConfig(motion::AxisId id)
{
    motion::AxisConfig cfg;
    cfg.id = id;
    cfg.min_phase_interval_us = CONFIG_MOTOR_MIN_STEP_INTERVAL_US;
    // Скорости осей и mapping берут значения из одних и тех же символов,
    // чтобы планировщик не ограничивал уставку молча.
    cfg.start_rate_steps_s = static_cast<float>(CONFIG_APP_AXIS_START_RATE_HALF_STEPS_S);
    cfg.max_rate_steps_s = static_cast<float>(CONFIG_APP_AXIS_MAX_RATE_HALF_STEPS_S);
    return cfg;
}

control::JoyMappingConfig makeJoyMapping()
{
    control::JoyMappingConfig cfg;
    cfg.x_axis = motion::AxisId::Azimuth;
    cfg.y_axis = motion::AxisId::Altitude;
    cfg.max_rate[0] = static_cast<float>(CONFIG_APP_AXIS_MAX_RATE_HALF_STEPS_S);
    cfg.max_rate[1] = static_cast<float>(CONFIG_APP_AXIS_MAX_RATE_HALF_STEPS_S);
    cfg.min_run_rate[0] = static_cast<float>(CONFIG_CTRL_JOY_MIN_RATE_HALF_STEPS_S);
    cfg.min_run_rate[1] = static_cast<float>(CONFIG_CTRL_JOY_MIN_RATE_HALF_STEPS_S);
    // Одна настройка мёртвой зоны на input_hal и control_hal.
    cfg.dead_zone = static_cast<float>(CONFIG_JOY_DEAD_ZONE_PERMILLE) / 1000.0F;
    cfg.stale_after_us = CONFIG_CTRL_AXIS_STALE_US;
    return cfg;
}

const char *axisName(motion::AxisId id)
{
    return id == motion::AxisId::Azimuth ? "azimuth" : "altitude";
}

bool isMoving(const motion::AxisStatus &status)
{
    return status.state == motion::AxisState::Accelerating ||
           status.state == motion::AxisState::Cruising ||
           status.state == motion::AxisState::Decelerating;
}

/**
 * @brief Ждёт завершения конкретного перемещения.
 *
 * Нулевой скорости недостаточно: сразу после постановки команды в очередь ось
 * ещё стоит, поэтому сначала подтверждаем начало движения, затем ждём
 * ожидаемую позицию без Fault.
 */
bool waitForMoveCompletion(motor_hal::MotorService &service, motion::AxisId id,
                           std::int64_t expected_position, std::uint32_t timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    const std::int64_t initial_position = service.status(id).position_step;
    bool accepted = false;

    while ((xTaskGetTickCount() - start) < timeout) {
        const motion::AxisStatus status = service.status(id);
        if (status.state == motion::AxisState::Fault || status.has_fault) {
            return false;
        }
        if (!accepted && (isMoving(status) || status.position_step != initial_position)) {
            accepted = true;
        }
        if (accepted && !isMoving(status) && status.rate_steps_s == 0.0F &&
            status.position_step == expected_position) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

void logJoystick(input_hal::JoystickService &service)
{
    const input_hal::JoystickStatus status = service.status();
    ESP_LOGI(kTag,
             "joystick: calibrated=%d sample=%d output=%d fault=%d neutral=%d "
             "x=%.3f y=%.3f raw=%u/%u last_valid_us=%llu errors=%lu events=%lu",
             status.calibrated ? 1 : 0, status.sample_valid ? 1 : 0,
             status.output_valid ? 1 : 0, status.read_fault ? 1 : 0,
             status.neutral ? 1 : 0,
             static_cast<double>(status.stick.normalized[0]),
             static_cast<double>(status.stick.normalized[1]),
             static_cast<unsigned>(status.raw.counts[0]),
             static_cast<unsigned>(status.raw.counts[1]),
             static_cast<unsigned long long>(status.last_valid_us),
             static_cast<unsigned long>(status.read_errors),
             static_cast<unsigned long>(status.button_events));
}

void logAxis(motor_hal::MotorService &service, motion::AxisId id)
{
    const motion::AxisStatus status = service.status(id);
    ESP_LOGI(kTag,
             "%s: state=%u pos=%lld rate=%.1f energized=%d trusted=%d "
             "late_corr=%llu late_replan=%llu stale=%llu rejected=%llu",
             axisName(id), static_cast<unsigned>(status.state),
             static_cast<long long>(status.position_step),
             static_cast<double>(status.rate_steps_s), status.energized ? 1 : 0,
             status.position_trusted ? 1 : 0,
             static_cast<unsigned long long>(status.late_corrections),
             static_cast<unsigned long long>(status.late_replans),
             static_cast<unsigned long long>(status.stale_events),
             static_cast<unsigned long long>(status.rejected_commands));
}

void logControl(control_hal::ControlService &control)
{
    const control_hal::ControlStatus status = control.status();
    ESP_LOGI(kTag,
             "control: phase=%s latch=%s owner=%s gen=%lu permit=%d recovery=%d "
             "desired=%.1f/%.1f enqueued=%.1f/%.1f confirmed=%.1f/%.1f "
             "joy_valid=%d joy_fresh=%d joy_neutral=%d joy_fault=%d "
             "delivery_fail=%lu/%lu stop_timeouts=%lu escalations=%lu "
             "ack_mismatch=%lu slot_busy=%lu unmatched_em=%lu notify_fail=%lu "
             "rejected=%lu cycles=%lu",
             state::phaseName(status.phase), state::latchName(status.latch),
             state::sourceName(status.owner),
             static_cast<unsigned long>(status.owner_generation),
             status.permit ? 1 : 0, status.recovery_active ? 1 : 0,
             static_cast<double>(status.desired_rate[0]),
             static_cast<double>(status.desired_rate[1]),
             static_cast<double>(status.enqueued_rate[0]),
             static_cast<double>(status.enqueued_rate[1]),
             static_cast<double>(status.confirmed_rate[0]),
             static_cast<double>(status.confirmed_rate[1]),
             status.joystick_valid ? 1 : 0, status.joystick_fresh ? 1 : 0,
             status.joystick_neutral ? 1 : 0, status.joystick_fault ? 1 : 0,
             static_cast<unsigned long>(status.delivery_failures[0]),
             static_cast<unsigned long>(status.delivery_failures[1]),
             static_cast<unsigned long>(status.stop_timeouts),
             static_cast<unsigned long>(status.escalations),
             static_cast<unsigned long>(status.ack_mismatches),
             static_cast<unsigned long>(status.emergency_slot_busy),
             static_cast<unsigned long>(status.unmatched_emergency_notifications),
             static_cast<unsigned long>(status.notify_failures),
             static_cast<unsigned long>(status.rejected_intents),
             static_cast<unsigned long>(status.cycles));
}

#if CONFIG_APP_MEMORY_DIAGNOSTICS

/**
 * @brief Счётчики heap по отдельным capability-наборам.
 *
 * Категории INTERNAL, DMA и SPIRAM пересекаются, поэтому печатаются
 * раздельно и не суммируются. Функция только читает счётчики аллокатора,
 * не удерживает моторные мьютексы и допустима во время движения.
 */
void logHeapSummary(const char *when)
{
    multi_heap_info_t info{};
    heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_LOGI(kTag,
             "heap[%s]: free=%u min_free=%u largest=%u allocated=%u "
             "blocks=%u/%u dma_free=%u spiram_free=%u",
             when,
             static_cast<unsigned>(info.total_free_bytes),
             static_cast<unsigned>(info.minimum_free_bytes),
             static_cast<unsigned>(info.largest_free_block),
             static_cast<unsigned>(info.total_allocated_bytes),
             static_cast<unsigned>(info.free_blocks),
             static_cast<unsigned>(info.total_blocks),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_DMA)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
}

/**
 * @brief Минимальный остаток стека одной задачи.
 *
 * Значение отражает минимум с момента запуска задачи. Единицы измерения
 * берутся из документации ESP-IDF для uxTaskGetStackHighWaterMark2 и перед
 * цитированием сверяются с заголовком v6.1.
 */
void logTaskStack(const char *when, const char *name, TaskHandle_t handle)
{
    if (handle == nullptr) {
        ESP_LOGW(kTag, "stack[%s]: task %s not found", when, name);
        return;
    }
    const auto free_units = uxTaskGetStackHighWaterMark2(handle);
    ESP_LOGI(kTag, "stack[%s]: %-10s free=%u", when, name,
             static_cast<unsigned>(free_units));
}

/**
 * @brief Снимок стеков. Только при подтверждённом отсутствии движения.
 *
 * Две задачи осей имеют одинаковое имя "motor_axis", поэтому по имени их не
 * различить: при включённом CONFIG_FREERTOS_USE_TRACE_FACILITY выводится
 * перечисление задач. Ёмкость массива уменьшать нельзя: при нехватке места
 * uxTaskGetSystemState вернёт 0, что считается ошибкой диагностики.
 */
void logStackSnapshotStopped(const char *when)
{
    // main измеряется напрямую из собственной задачи: поиск по имени не нужен.
    logTaskStack(when, "main", nullptr);
    logTaskStack(when, "joystick", xTaskGetHandle("joystick"));
    logTaskStack(when, "control", xTaskGetHandle("control"));
    logTaskStack(when, "wifi_ap", xTaskGetHandle("wifi_ap"));

#if CONFIG_FREERTOS_USE_TRACE_FACILITY
    constexpr UBaseType_t kTaskCapacity = 24;
    TaskStatus_t tasks[kTaskCapacity]{};
    const UBaseType_t count = uxTaskGetSystemState(tasks, kTaskCapacity, nullptr);
    if (count == 0) {
        ESP_LOGE(kTag, "stack[%s]: task enumeration failed", when);
        return;
    }
    for (UBaseType_t i = 0; i < count; ++i) {
        if (tasks[i].pcTaskName == nullptr) {
            continue;
        }
        ESP_LOGI(kTag, "stack[%s]: %-12s state=%u free=%u", when,
                 tasks[i].pcTaskName,
                 static_cast<unsigned>(tasks[i].eCurrentState),
                 static_cast<unsigned>(tasks[i].usStackHighWaterMark));
    }
    if (count == kTaskCapacity) {
        ESP_LOGW(kTag, "stack[%s]: task list may be truncated", when);
    }
#else
    ESP_LOGW(kTag,
             "stack[%s]: axes not listed, enable CONFIG_FREERTOS_USE_TRACE_FACILITY",
             when);
#endif
}

/**
 * @brief Точные размеры объектов на целевой ABI.
 *
 * Значения нужны для сверки бюджета main stack и .bss: host-размеры для
 * Xtensa непригодны. Единицы — байты.
 */
void logObjectSizes()
{
    ESP_LOGI(kTag,
             "sizeof: MotorService=%u JoystickService=%u ControlService=%u "
             "StateHub=%u SystemSnapshot=%u AxisStatus=%u semaphore=%u",
             static_cast<unsigned>(sizeof(motor_hal::MotorService)),
             static_cast<unsigned>(sizeof(input_hal::JoystickService)),
             static_cast<unsigned>(sizeof(control_hal::ControlService)),
             static_cast<unsigned>(sizeof(state_hal::StateHub)),
             static_cast<unsigned>(sizeof(state::SystemSnapshot)),
             static_cast<unsigned>(sizeof(motion::AxisStatus)),
             static_cast<unsigned>(sizeof(StaticSemaphore_t)));
}

#endif  // CONFIG_APP_MEMORY_DIAGNOSTICS

#if CONFIG_APP_MOTOR_SELFTEST

void runSelfTest(motor_hal::MotorService &service)
{
    ESP_LOGW(kTag, "self-test enabled: both axes will move");

    (void)service.enable(motion::AxisId::Azimuth);
    (void)service.enable(motion::AxisId::Altitude);

    // Азимут: два оборота с разгоном и плавным остановом.
    const std::int64_t azimuth_start = service.status(motion::AxisId::Azimuth).position_step;
    (void)service.moveSteps(motion::AxisId::Azimuth, 2 * kHalfStepsPerRevolution);
    if (!waitForMoveCompletion(service, motion::AxisId::Azimuth,
                               azimuth_start + 2 * kHalfStepsPerRevolution, 30000)) {
        ESP_LOGE(kTag, "azimuth move did not complete");
        (void)service.stopAxis(motion::AxisId::Azimuth, motion::StopMode::Immediate);
    }
    logAxis(service, motion::AxisId::Azimuth);

    // Высота: то же в обратном направлении.
    const std::int64_t altitude_start = service.status(motion::AxisId::Altitude).position_step;
    (void)service.moveSteps(motion::AxisId::Altitude, -2 * kHalfStepsPerRevolution);
    if (!waitForMoveCompletion(service, motion::AxisId::Altitude,
                               altitude_start - 2 * kHalfStepsPerRevolution, 30000)) {
        ESP_LOGE(kTag, "altitude move did not complete");
        (void)service.stopAxis(motion::AxisId::Altitude, motion::StopMode::Immediate);
    }
    logAxis(service, motion::AxisId::Altitude);

    // Одновременное движение и плавный останов.
    (void)service.setTargetRate(motion::AxisId::Azimuth, 400.0F);
    (void)service.setTargetRate(motion::AxisId::Altitude, -400.0F);
    vTaskDelay(pdMS_TO_TICKS(1000));
    (void)service.stopAll(motion::StopMode::Smooth);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Аварийный останов: задержка от запроса до снятия фаз.
    (void)service.setTargetRate(motion::AxisId::Azimuth, 600.0F);
    vTaskDelay(pdMS_TO_TICKS(300));
    const std::int64_t requested_us = esp_timer_get_time();
    (void)service.stopAll(motion::StopMode::Immediate);
    bool deenergized = false;
    std::int64_t released_us = 0;
    for (int i = 0; i < 200; ++i) {
        const motion::AxisStatus status = service.status(motion::AxisId::Azimuth);
        if (!isMoving(status) && !status.energized) {
            released_us = esp_timer_get_time();
            deenergized = true;
            break;
        }
        vTaskDelay(1);
    }
    if (deenergized) {
        ESP_LOGW(kTag, "emergency stop latency: %lld us",
                 static_cast<long long>(released_us - requested_us));
    } else {
        ESP_LOGE(kTag, "emergency stop: de-energizing not confirmed within 200 ms");
    }
    logAxis(service, motion::AxisId::Azimuth);
    logAxis(service, motion::AxisId::Altitude);
}

#endif  // CONFIG_APP_MOTOR_SELFTEST

}  // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "firmware starting");
    ESP_LOGI(kTag, "ESP-IDF version: %s", esp_get_idf_version());

    logChipInfo();

    ESP_ERROR_CHECK(board::init());

    // Точка доступа стартует до приводов: инициализация NVS и
    // RF-калибровка выполняются до любого движения. Объект имеет
    // статическое время жизни, поэтому деструктор не вызывается.
    static wifi_hal::WifiApService access_point;
    const esp_err_t ap_err = access_point.start();
    if (ap_err != ESP_OK) {
        ESP_LOGE(kTag, "access point unavailable: %s",
                 esp_err_to_name(ap_err));
    }

    motor_hal::MotorService service;
    const motion::AxisConfig configs[2] = {
        makeAxisConfig(motion::AxisId::Azimuth),
        makeAxisConfig(motion::AxisId::Altitude),
    };
    ESP_ERROR_CHECK(service.start(board::kMotorPins, configs));
    ESP_LOGI(kTag, "motion service started: half-step units, GPTimer step clock");

#if CONFIG_APP_MOTOR_SELFTEST
    runSelfTest(service);
#endif

    // Запуск после самотеста: синхронная калибровка до 10 с не задерживает
    // проверку приводов. Владелец движения передаётся управляющей службе
    // только здесь, поэтому одновременное управление невозможно.
    static state_hal::StateHub hub;
    ButtonStatsListener button_listener(access_point);
    input_hal::JoystickService joystick;
    const esp_err_t joystick_err =
        joystick.start(board::kJoystickPins, &button_listener);
    if (joystick_err != ESP_OK) {
        ESP_LOGE(kTag, "joystick unavailable: %s",
                 esp_err_to_name(joystick_err));
    }

    control_hal::ControlService control(service, joystick, hub);
    if (joystick_err == ESP_OK) {
        const esp_err_t control_err = control.start(makeJoyMapping());
        if (control_err != ESP_OK) {
            ESP_LOGE(kTag, "control unavailable: %s",
                     esp_err_to_name(control_err));
        } else {
            const esp_err_t arm_err = control.arm();
            if (arm_err != ESP_OK) {
                // Защёлка сохранена: движение запрещено до acknowledgeFault.
                ESP_LOGW(kTag, "control armed with latch: %s",
                         esp_err_to_name(arm_err));
            }
        }
    }

#if CONFIG_APP_MEMORY_DIAGNOSTICS
    // Все службы запущены и движения нет: снимок стеков допустим только здесь.
    logHeapSummary("boot");
    logObjectSizes();
    logStackSnapshotStopped("boot");
#endif

    while (true) {
        ESP_LOGI(kTag, "heartbeat");
        logAxis(service, motion::AxisId::Azimuth);
        logAxis(service, motion::AxisId::Altitude);
        logJoystick(joystick);
        logControl(control);
        logButtonStats(button_listener.stats());
        logAccessPoint(access_point.status());
#if CONFIG_APP_MEMORY_DIAGNOSTICS
        // Только чтение счётчиков: допустимо и во время движения.
        logHeapSummary("periodic");
#endif
        vTaskDelay(pdMS_TO_TICKS(CONFIG_APP_STATUS_PERIOD_MS));
    }
}
