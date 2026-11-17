/**
 * @file control_service.cpp
 * @brief Управляющая задача: цикл, протокол останова, восстановление.
 */

#include "control_hal/control_service.hpp"

#include <cstddef>

#include "control/joy_observation.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

namespace control_hal {
namespace {

constexpr const char *kTag = "control";
constexpr std::uint32_t kStopBit = 1U << 0;
constexpr std::uint32_t kRecoveryBit = 1U << 1;
constexpr std::uint32_t kRecoveryStepMs = 5;

using state::Micros;

TickType_t msToTicks(std::uint32_t ms) noexcept
{
    const TickType_t ticks = pdMS_TO_TICKS(ms);
    return ticks == 0 ? 1 : ticks;
}

bool isMoving(const motion::AxisStatus &status) noexcept
{
    return status.state == motion::AxisState::Accelerating ||
           status.state == motion::AxisState::Cruising ||
           status.state == motion::AxisState::Decelerating;
}

bool restOrCannotMove(const motion::AxisStatus &status) noexcept
{
    return control::StopProtocol::axisCannotMove(status) ||
           control::StopProtocol::axisAtRest(status);
}

/// Перенос полей статуса сервиса в чистый вход построителя наблюдения.
control::JoystickSnapshotInput toSnapshotInput(
    const input_hal::JoystickStatus &status) noexcept
{
    control::JoystickSnapshotInput input{};
    input.calibrated = status.calibrated;
    input.sample_valid = status.sample_valid;
    input.output_valid = status.output_valid;
    input.read_fault = status.read_fault;
    input.neutral = status.neutral;
    input.stick = status.stick;
    input.raw = status.raw;
    input.measured_us = status.last_valid_us;
    input.read_errors = status.read_errors;
    return input;
}

}  // namespace

ControlService::ControlService(motor_hal::MotorService &motors,
                               input_hal::JoystickService &joystick,
                               state_hal::StateHub &hub)
    : motors_(motors), joystick_(joystick), hub_(hub), port_(motors)
{
    api_mutex_ = xSemaphoreCreateMutexStatic(&api_mutex_storage_);
    cal_mutex_ = xSemaphoreCreateMutexStatic(&cal_mutex_storage_);
    recovery_mutex_ = xSemaphoreCreateMutexStatic(&recovery_mutex_storage_);
    exit_sem_ = xSemaphoreCreateBinaryStatic(&exit_sem_storage_);
    recovery_done_ = xSemaphoreCreateBinaryStatic(&recovery_done_storage_);
}

ControlService::~ControlService()
{
    while (stop() == ESP_ERR_TIMEOUT) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

Micros ControlService::nowUs() const noexcept
{
    return static_cast<Micros>(esp_timer_get_time());
}

std::size_t ControlService::indexOf(motion::AxisId id) noexcept
{
    return static_cast<std::size_t>(id);
}

bool ControlService::lockApi(TickType_t wait) const
{
    return api_mutex_ != nullptr && xSemaphoreTake(api_mutex_, wait) == pdTRUE;
}

void ControlService::unlockApi() const
{
    (void)xSemaphoreGive(api_mutex_);
}

bool ControlService::lockCal(TickType_t wait) const
{
    return cal_mutex_ != nullptr && xSemaphoreTake(cal_mutex_, wait) == pdTRUE;
}

void ControlService::unlockCal() const
{
    (void)xSemaphoreGive(cal_mutex_);
}

void ControlService::setLatch(state::LatchCode latch) noexcept
{
    (void)xSemaphoreTake(recovery_mutex_, portMAX_DELAY);
    latch_ = latch;
    (void)xSemaphoreGive(recovery_mutex_);
}

state::LatchCode ControlService::latch() const noexcept
{
    if (recovery_mutex_ == nullptr) {
        return state::LatchCode::None;
    }
    (void)xSemaphoreTake(recovery_mutex_, portMAX_DELAY);
    const state::LatchCode value = latch_;
    (void)xSemaphoreGive(recovery_mutex_);
    return value;
}

void ControlService::markRecoveryCancelledIfPending() noexcept
{
    if (recovery_mutex_ == nullptr) {
        return;
    }
    // Короткая критическая секция: согласует отмену с терминальным переходом
    // финализатора. Порт не вызывается, семафоры не ожидаются, execution_busy
    // не изменяется, уже зафиксированный результат не переписывается.
    (void)xSemaphoreTake(recovery_mutex_, portMAX_DELAY);
    if (recovery_.state == RecoveryState::Pending) {
        recovery_cancel_requested_ = true;
    }
    (void)xSemaphoreGive(recovery_mutex_);
}

bool ControlService::axesHaveFault() const noexcept
{
    for (std::size_t i = 0; i < 2; ++i) {
        if (axis_[i].has_fault || axis_[i].state == motion::AxisState::Fault) {
            return true;
        }
    }
    return false;
}

bool ControlService::axesReady() const noexcept
{
    for (std::size_t i = 0; i < 2; ++i) {
        if (axis_[i].has_fault || axis_[i].state == motion::AxisState::Disabled) {
            return false;
        }
    }
    return true;
}

bool ControlService::stoppedConfirmed() const noexcept
{
    return stop_protocol_.active() && stop_protocol_.confirmed();
}

bool ControlService::calibrationInputReady(Micros now) const noexcept
{
    const std::uint32_t session = calibration_ready_session_.load();
    const Micros until = calibration_ready_until_.load();
    return calibration_ready_.load() && session != 0U && until != 0U && now <= until;
}

void ControlService::revokeMotionPermission() noexcept
{
    motion_generation_.fetch_add(1);
    calibration_permission_active_ = false;
}

esp_err_t ControlService::requestCalibrationStop(std::uint64_t request_id)
{
    if (request_id == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    // Идемпотентность: повтор с тем же или новым id не создаёт вторую
    // логическую операцию, пока запрос не обработан.
    std::uint64_t expected = 0;
    if (!pending_calibration_stop_.compare_exchange_strong(expected, request_id)) {
        return ESP_OK;
    }
    // Не safety stop: владение и разрешение сохраняются до подтверждения.
    calibration_stop_requested_.store(true);
    return ESP_OK;
}

CalibrationStopAttestation ControlService::lastStopAttestation() const
{
    CalibrationStopAttestation copy{};
    if (!lockCal(portMAX_DELAY)) {
        return copy;
    }
    copy = attestation_;
    unlockCal();
    return copy;
}

esp_err_t ControlService::beginCalibrationHold(motion::AxisId axis, std::uint32_t session,
                                               Micros absolute_deadline_us)
{
    const std::size_t index = indexOf(axis);
    if (index >= 2 || session == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!lockCal(portMAX_DELAY)) {
        return ESP_ERR_INVALID_STATE;
    }
    CalibrationHold &hold = calibration_hold_[index];
    if (hold.active && hold.session == session) {
        // Повтор не перезаписывает исходную политику и не расширяет предел.
        if (absolute_deadline_us != 0U &&
            (hold.deadline == 0U || absolute_deadline_us < hold.deadline)) {
            hold.deadline = absolute_deadline_us;
        }
        unlockCal();
        return ESP_OK;
    }
    HoldRequest &request = hold_request_[index];
    request.pending = true;
    request.begin = true;
    request.session = session;
    request.deadline = absolute_deadline_us;
    unlockCal();
    return ESP_OK;
}

esp_err_t ControlService::endCalibrationHold(motion::AxisId axis, std::uint32_t session)
{
    const std::size_t index = indexOf(axis);
    if (index >= 2 || session == 0U) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!lockCal(portMAX_DELAY)) {
        return ESP_ERR_INVALID_STATE;
    }
    HoldRequest &request = hold_request_[index];
    request.pending = true;
    request.begin = false;
    request.session = session;
    request.deadline = 0;
    unlockCal();
    return ESP_OK;
}

bool ControlService::calibrationHoldActive(motion::AxisId axis, std::uint32_t session) const
{
    const std::size_t index = indexOf(axis);
    if (index >= 2 || !lockCal(10)) {
        return false;
    }
    const bool active = calibration_hold_[index].active &&
                        calibration_hold_[index].session == session;
    unlockCal();
    return active;
}

bool ControlService::calibrationHoldCleared(motion::AxisId axis) const
{
    const std::size_t index = indexOf(axis);
    if (index >= 2 || !lockCal(10)) {
        // Ошибка чтения не выдаётся за подтверждённое освобождение.
        return false;
    }
    const CalibrationHold &hold = calibration_hold_[index];
    const bool cleared = !hold.active && !hold.restore_pending;
    unlockCal();
    return cleared;
}

motion::HoldPolicy ControlService::holdPolicy(motion::AxisId axis) const
{
    return port_.holdPolicy(axis);
}

void ControlService::setCalibrationInputReady(bool ready, std::uint32_t session, Micros valid_until_us)
{
    if (!ready) {
        calibration_ready_.store(false);
        calibration_ready_until_.store(0);
        return;
    }
    if (session == 0U || valid_until_us == 0U) {
        return;
    }
    calibration_ready_session_.store(session);
    calibration_ready_until_.store(valid_until_us);
    calibration_ready_.store(true);
}

std::uint32_t ControlService::motionGeneration() const
{
    return motion_generation_.load();
}

esp_err_t ControlService::beginFlashWindow(Micros timeout_us)
{
    if (flash_window_active_.load()) {
        return ESP_OK;
    }
    const Micros deadline = nowUs() + (timeout_us == 0U ? 10'000'000U : timeout_us);
    flash_window_requested_us_.store(deadline);
    flash_window_requested_.store(true);
    return ESP_OK;
}

void ControlService::endFlashWindow()
{
    flash_window_requested_.store(false);
    flash_window_active_.store(false);
}

bool ControlService::flashWindowActive() const
{
    return flash_window_active_.load();
}

bool ControlService::calibrationInputReady() const
{
    return calibrationInputReady(nowUs());
}

esp_err_t ControlService::start(const control::JoyMappingConfig &mapping)
{
    if (!lockApi(portMAX_DELAY)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (lifecycle_ != Lifecycle::Stopped) {
        unlockApi();
        return ESP_ERR_INVALID_STATE;
    }
    if (!control::isValid(mapping)) {
        ESP_LOGE(kTag, "invalid joystick mapping");
        unlockApi();
        return ESP_ERR_INVALID_ARG;
    }

    control::InputGracePolicy::Config grace_config{};
    grace_config.grace_us = CONFIG_CTRL_INPUT_GRACE_US;
    if (!grace_.configure(grace_config, CONFIG_CTRL_AXIS_STALE_US)) {
        unlockApi();
        return ESP_ERR_INVALID_ARG;
    }

    (void)xSemaphoreTake(recovery_mutex_, portMAX_DELAY);
    const bool record_in_use = recovery_.result_unread || recovery_.execution_busy ||
                               recovery_.state == RecoveryState::Pending;
    if (!record_in_use) {
        recovery_ = RecoveryRecord{};
    }
    (void)xSemaphoreGive(recovery_mutex_);
    if (record_in_use) {
        ESP_LOGE(kTag, "recovery record is still in use");
        unlockApi();
        return ESP_ERR_INVALID_STATE;
    }

    mapping_ = mapping;

    control::RateGate::Config gate_config{};
    gate_config.deadband = static_cast<float>(CONFIG_CTRL_RATE_DEADBAND_HALF_STEPS_S);
    gate_.configure(gate_config);

    control::StopProtocol::Config stop_config{};
    stop_config.confirm_timeout_us = CONFIG_CTRL_STOP_CONFIRM_TIMEOUT_US;
    stop_protocol_.configure(stop_config);

    control::ArmingConfig arming_config{};
    arming_config.delivery_failure_limit = CONFIG_CTRL_DELIVERY_FAILURE_LIMIT;
    arming_.configure(arming_config);
    const state::LatchCode saved_latch = latch();
    if (saved_latch != state::LatchCode::None) {
        arming_.restoreLatch(saved_latch);  // защёлка переживает перезапуск
    }

    control::Arbiter::Config arbiter_config{};
    arbiter_config.default_lease_us = CONFIG_CTRL_INTENT_LEASE_US;
    arbiter_config.joystick_idle_release_us =
        static_cast<Micros>(CONFIG_CTRL_JOY_IDLE_RELEASE_MS) * 1000U;
    arbiter_.configure(arbiter_config);

    inbox_.clear();
    external_.reset();

    permit_requested_.store(false);
    safety_stop_requested_.store(false);
    calibration_stop_requested_.store(false);
    shutdown_requested_.store(false);
    stop_failed_.store(false);

    (void)xSemaphoreTake(cal_mutex_, portMAX_DELAY);
    for (std::size_t i = 0; i < 2; ++i) {
        hold_request_[i] = HoldRequest{};
        calibration_hold_[i] = CalibrationHold{};
    }
    attestation_ = CalibrationStopAttestation{};
    (void)xSemaphoreGive(cal_mutex_);
    calibration_ready_.store(false);
    calibration_ready_session_.store(0);
    calibration_ready_until_.store(0);
    motion_generation_.store(0);
    pending_calibration_stop_.store(0);
    flash_window_requested_.store(false);
    flash_window_requested_us_.store(0);
    flash_window_active_.store(false);
    calibration_permission_active_ = false;
    calibration_ready_seen_ = false;
    require_neutral_after_calibration_ = false;
    calibration_released_us_ = 0;

    for (std::size_t i = 0; i < 2; ++i) {
        axis_polled_[i] = 0;
        axis_read_failures_[i] = 0;
        baseline_epoch_[i] = 0;
        delivery_failures_[i] = 0;
        enable_failures_[i] = 0;
        enable_accepted_[i] = true;
        last_stop_ticket_[i] = 0;
    }
    stop_timeouts_ = 0;
    escalations_ = 0;
    ack_mismatches_ = 0;
    rejected_intents_ = 0;
    cycles_ = 0;
    joystick_sequence_ = 0;
    recovery_timeouts_ = 0;
    recovery_cancelled_ = 0;
    recovery_failed_ = 0;
    recovery_late_ = 0;
    stop_failed_flag_ = false;
    stop_confirmed_flag_ = false;
    enable_pending_ = false;
    neutral_tracking_ = false;
    neutral_since_ = 0;
    input_usable_ = false;
    conversion_ = control::JoyConversion{};

    period_ticks_ = msToTicks(CONFIG_CTRL_PERIOD_MS);
    (void)xSemaphoreTake(exit_sem_, 0);
    (void)xSemaphoreTake(recovery_done_, 0);

    const BaseType_t created = xTaskCreatePinnedToCore(
        &ControlService::taskEntry, "control", CONFIG_CTRL_TASK_STACK, this,
        CONFIG_CTRL_TASK_PRIORITY, &task_, CONFIG_CTRL_TASK_CORE);
    if (created != pdPASS) {
        task_ = nullptr;
        unlockApi();
        return ESP_ERR_NO_MEM;
    }

    lifecycle_ = Lifecycle::Running;
    unlockApi();
    return ESP_OK;
}

esp_err_t ControlService::stop()
{
    if (!lockApi(portMAX_DELAY)) {
        return ESP_OK;
    }
    if (lifecycle_ == Lifecycle::Stopped) {
        unlockApi();
        return ESP_OK;
    }
    if (lifecycle_ == Lifecycle::Running) {
        lifecycle_ = Lifecycle::Stopping;
        shutdown_requested_.store(true);
        markRecoveryCancelledIfPending();  // отмена и успех согласованы
        if (task_ != nullptr) {
            (void)xTaskNotify(task_, kStopBit, eSetBits);
        }
    }

    const TickType_t timeout = msToTicks(CONFIG_CTRL_STOP_TIMEOUT_MS);
    if (task_ != nullptr && xSemaphoreTake(exit_sem_, timeout) != pdTRUE) {
        ESP_LOGE(kTag, "control task did not confirm shutdown");
        unlockApi();
        return ESP_ERR_TIMEOUT;  // lifecycle остаётся Stopping
    }

    task_ = nullptr;
    lifecycle_ = Lifecycle::Stopped;
    hub_.invalidateAxis(motion::AxisId::Azimuth);
    hub_.invalidateAxis(motion::AxisId::Altitude);
    hub_.invalidateJoystick();
    const bool failed = stop_failed_.load();
    unlockApi();
    return failed ? ESP_FAIL : ESP_OK;
}

esp_err_t ControlService::arm()
{
    if (!lockApi(portMAX_DELAY)) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t result = ESP_OK;
    if (lifecycle_ != Lifecycle::Running) {
        result = ESP_ERR_INVALID_STATE;
    } else if (latch() != state::LatchCode::None) {
        result = ESP_ERR_INVALID_STATE;
    } else {
        permit_requested_.store(true);
        if (task_ != nullptr) {
            (void)xTaskNotify(task_, kRecoveryBit, eSetBits);
        }
    }
    unlockApi();
    return result;
}

esp_err_t ControlService::disarm()
{
    if (!lockApi(portMAX_DELAY)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (lifecycle_ != Lifecycle::Running) {
        unlockApi();
        return ESP_ERR_INVALID_STATE;
    }
    permit_requested_.store(false);
    if (task_ != nullptr) {
        (void)xTaskNotify(task_, kRecoveryBit, eSetBits);
    }
    unlockApi();
    return ESP_OK;
}

esp_err_t ControlService::requestSafetyStop()
{
    if (!lockApi(portMAX_DELAY)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (lifecycle_ != Lifecycle::Running) {
        unlockApi();
        return ESP_ERR_INVALID_STATE;
    }
    safety_stop_requested_.store(true);
    markRecoveryCancelledIfPending();  // приоритет выше восстановления
    if (task_ != nullptr) {
        (void)xTaskNotify(task_, kStopBit, eSetBits);
    }
    unlockApi();
    return ESP_OK;
}

esp_err_t ControlService::submitIntent(const control::Intent &intent)
{
    if (!lockApi(0)) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t result = ESP_ERR_INVALID_STATE;
    if (lifecycle_ == Lifecycle::Running) {
        result = external_.submit(intent);
    }
    unlockApi();
    return result;
}

esp_err_t ControlService::acknowledgeFault(state::Micros timeout_us)
{
    if (!lockApi(portMAX_DELAY)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (lifecycle_ != Lifecycle::Running) {
        unlockApi();
        return ESP_ERR_INVALID_STATE;
    }
    const Micros now = nowUs();
    const Micros limit = timeout_us != 0U
                             ? timeout_us
                             : static_cast<Micros>(CONFIG_CTRL_ACK_TIMEOUT_MS) * 1000U;

    std::uint32_t id = 0;
    (void)xSemaphoreTake(recovery_mutex_, portMAX_DELAY);
    const bool busy = recovery_.execution_busy || recovery_.result_unread ||
                      recovery_.state == RecoveryState::Pending;
    if (!busy) {
        id = next_recovery_id_++;
        recovery_.state = RecoveryState::Pending;
        recovery_.request_id = id;
        recovery_.deadline_us = now + limit;
        recovery_.execution_busy = false;
        recovery_.result_unread = true;
        recovery_.result = ESP_OK;
        recovery_cancel_requested_ = false;
    }
    (void)xSemaphoreGive(recovery_mutex_);
    if (id == 0U) {
        unlockApi();
        return ESP_ERR_INVALID_STATE;
    }
    if (task_ != nullptr) {
        (void)xTaskNotify(task_, kRecoveryBit, eSetBits);
    }
    unlockApi();

    const Micros deadline = now + limit;
    esp_err_t result = ESP_ERR_TIMEOUT;
    for (;;) {
        const Micros current = nowUs();
        const bool before_deadline = current < deadline;
        const TickType_t wait =
            before_deadline ? msToTicks(static_cast<std::uint32_t>((deadline - current) / 1000U) + 1U)
                            : 0;
        (void)xSemaphoreTake(recovery_done_, wait);

        bool done = false;
        (void)xSemaphoreTake(recovery_mutex_, portMAX_DELAY);
        if (recovery_.request_id == id) {
            switch (recovery_.state) {
                case RecoveryState::Succeeded:
                    recovery_.result_unread = false;
                    result = recovery_.result;
                    done = true;
                    break;
                case RecoveryState::Failed:
                case RecoveryState::Cancelled:
                    recovery_.result_unread = false;
                    result = recovery_.result;
                    done = true;
                    break;
                case RecoveryState::Pending:
                    if (nowUs() >= deadline) {
                        recovery_.state = RecoveryState::Cancelled;
                        recovery_.result = ESP_ERR_TIMEOUT;
                        recovery_.result_unread = false;
                        ++recovery_timeouts_;
                        result = ESP_ERR_TIMEOUT;
                        done = true;
                    }
                    break;
                case RecoveryState::Idle:
                    done = true;
                    break;
            }
        } else {
            done = true;  // запись переиспользована быть не должна
        }
        (void)xSemaphoreGive(recovery_mutex_);
        if (done) {
            break;
        }
    }
    return result;
}

ControlStatus ControlService::status() const
{
    const state::SystemSnapshot snapshot = hub_.snapshot();
    const state::ControlObservation &control = snapshot.control;
    ControlStatus out{};
    out.phase = control.phase;
    out.latch = control.latch;
    out.owner = control.owner;
    out.owner_generation = control.owner_generation;
    out.permit = control.permit;
    out.recovery_active = control.recovery_active;
    out.deny = control.deny;
    for (std::size_t i = 0; i < 2; ++i) {
        out.desired_rate[i] = control.desired_rate[i];
        out.enqueued_rate[i] = control.enqueued_rate[i];
        out.confirmed_rate[i] = control.confirmed_rate[i];
        out.delivery_failures[i] = control.delivery_failures[i];
    }
    out.joystick_valid = snapshot.joystick.meta.valid;
    out.joystick_fault = snapshot.joystick.read_fault;
    out.joystick_neutral = snapshot.joystick.neutral;
    out.joystick_fresh = state::freshness(snapshot.generated_us,
                                           snapshot.joystick.meta.measured_us,
                                           CONFIG_CTRL_AXIS_STALE_US) ==
                         state::Freshness::Fresh;
    out.stop_timeouts = control.stop_timeouts;
    out.escalations = control.escalations;
    out.ack_mismatches = control.ack_mismatches;
    out.emergency_slot_busy = control.emergency_slot_busy;
    out.unmatched_emergency_notifications = control.unmatched_emergency_notifications;
    out.notify_failures = control.notify_failures;
    out.rejected_intents = control.rejected_intents;
    out.cycles = static_cast<std::uint32_t>(control.cycle);
    return out;
}

void ControlService::taskEntry(void *arg)
{
    static_cast<ControlService *>(arg)->run();
}

void ControlService::run()
{
    // Baseline эпох: подтверждение возможно только для текущего запуска.
    for (std::size_t i = 0; i < 2; ++i) {
        const motion::AxisId id = static_cast<motion::AxisId>(i);
        axis_[i] = port_.status(id);
        baseline_epoch_[i] = axis_[i].command_epoch;
        axis_polled_[i] = nowUs();
    }

    for (;;) {
        std::uint32_t bits = 0;
        (void)xTaskNotifyWait(0, 0xFFFFFFFFU, &bits, period_ticks_);
        if ((bits & kStopBit) != 0U && shutdown_requested_.load()) {
            break;
        }
        cycle();
    }

    // Незавершённый запрос восстановления прерывается shutdown: защёлка
    // сохраняется, ожидающий получает ESP_ERR_INVALID_STATE.
    (void)xSemaphoreTake(recovery_mutex_, portMAX_DELAY);
    if (recovery_.state == RecoveryState::Pending) {
        recovery_.state = RecoveryState::Cancelled;
        recovery_.result = ESP_ERR_INVALID_STATE;
        recovery_.execution_busy = false;
        ++recovery_cancelled_;
    }
    recovery_cancel_requested_ = false;
    (void)xSemaphoreGive(recovery_mutex_);
    if (recovery_done_ != nullptr) {
        (void)xSemaphoreGive(recovery_done_);
    }

    finishShutdown();
    if (exit_sem_ != nullptr) {
        (void)xSemaphoreGive(exit_sem_);
    }
    vTaskDelete(nullptr);
}

void ControlService::pollInputs(Micros now)
{
    for (std::size_t i = 0; i < 2; ++i) {
        const motion::AxisId id = static_cast<motion::AxisId>(i);
        axis_polled_[i] = now;
        axis_[i] = port_.status(id);
        hub_.publishAxis(id, axis_[i], axis_polled_[i], axis_read_failures_[i]);
    }
    // meta.valid выставляет построитель: без него convert() вернёт NoSample,
    // хотя хаб уже публикует снимок.
    joystick_obs_ = control::makeObservation(toSnapshotInput(joystick_.status()), now);
    hub_.publishJoystick(joystick_obs_);
}

void ControlService::applyExternalIntents(Micros now)
{
    external_.drainInto(inbox_, now);

    control::IntentInbox::StopRequest stop{};
    if (inbox_.takeStop(stop)) {
        safety_stop_requested_.store(true);
    }

    control::IntentInbox::Event events[control::IntentInbox::kEventCapacity]{};
    const std::size_t event_count = inbox_.drainEvents(events, control::IntentInbox::kEventCapacity);
    for (std::size_t i = 0; i < event_count; ++i) {
        const state::SourceId before = arbiter_.owner();
        control::DenyReason deny = control::DenyReason::None;
        arbiter_.applyOwnership(events[i].source,
                                events[i].kind == control::IntentInbox::EventKind::Claim,
                                events[i].kind == control::IntentInbox::EventKind::Release,
                                events[i].session, now, deny);
        if (deny != control::DenyReason::None) {
            ++rejected_intents_;
        }
        if (before == state::SourceId::Calibration &&
            arbiter_.owner() != state::SourceId::Calibration) {
            // Возврат джойстику требует новой нейтрали после освобождения.
            require_neutral_after_calibration_ = true;
            calibration_released_us_ = now;
        }
    }

    control::Intent intents[state::kSourceCount]{};
    const std::size_t rate_count = inbox_.drainRates(intents, state::kSourceCount);
    for (std::size_t i = 0; i < rate_count; ++i) {
        arbiter_.applyRates(intents[i].source, intents[i].axis, intents[i].session,
                            intents[i].created_us, intents[i].valid_until_us);
    }
}

void ControlService::applyJoystick(Micros now)
{
    conversion_ = control::convert(joystick_obs_, mapping_, now);

    float desired[2] = {0.0F, 0.0F};
    if (conversion_.ok) {
        desired[0] = conversion_.rate[0];
        desired[1] = conversion_.rate[1];
        input_usable_ = true;
        grace_.onGoodSample(joystick_obs_.meta.measured_us);
    } else {
        // Свежесть проверяется независимо от раннего выхода convert().
        const bool stale =
            state::freshness(now, joystick_obs_.meta.measured_us,
                             CONFIG_CTRL_AXIS_STALE_US) != state::Freshness::Fresh;
        const control::InputGracePolicy::Decision decision =
            grace_.onRejected(stale ? state::DenyReason::StaleInput : conversion_.deny, now);
        input_usable_ = false;
        if (decision.hold_previous) {
            // Удерживаем только ранее УСПЕШНО поставленную ненулевую уставку.
            for (std::size_t i = 0; i < 2; ++i) {
                const motion::AxisId id = static_cast<motion::AxisId>(i);
                if (gate_.hasEnqueuedNonZero(id)) {
                    desired[i] = gate_.lastEnqueued(id);
                    input_usable_ = true;
                }
            }
        }
    }

    control::AxisRate rates[2]{};
    for (std::size_t i = 0; i < 2; ++i) {
        rates[i].valid = input_usable_;
        rates[i].half_steps_per_s = desired[i];
    }

    const bool neutral = joystick_obs_.neutral || !conversion_.ok;
    bool release = false;
    if (neutral) {
        if (!neutral_tracking_) {
            neutral_tracking_ = true;
            neutral_since_ = now;
        }
        if (CONFIG_CTRL_JOY_IDLE_RELEASE_MS > 0 &&
            arbiter_.owner() == state::SourceId::Joystick &&
            now - neutral_since_ >=
                static_cast<Micros>(CONFIG_CTRL_JOY_IDLE_RELEASE_MS) * 1000U) {
            release = true;
            neutral_tracking_ = false;
        }
    } else {
        neutral_tracking_ = false;
    }

    if (require_neutral_after_calibration_) {
        const bool fresh = state::freshness(now, joystick_obs_.meta.measured_us,
                                            CONFIG_CTRL_AXIS_STALE_US) ==
                           state::Freshness::Fresh;
        if (neutral && fresh && joystick_obs_.meta.measured_us > calibration_released_us_) {
            require_neutral_after_calibration_ = false;
        }
    }

    const bool claim = !require_neutral_after_calibration_ && !neutral && !release &&
                       arbiter_.owner() != state::SourceId::Joystick &&
                       !arbiter_.handoverPending();
    control::DenyReason deny = control::DenyReason::None;
    arbiter_.applyOwnership(state::SourceId::Joystick, claim, release, 1, now, deny);
    if (deny != control::DenyReason::None) {
        ++rejected_intents_;
    }
    if (require_neutral_after_calibration_ && !neutral && !release) {
        // Джойстик отклонён до новой нейтрали: это наблюдаемо.
        ++rejected_intents_;
    }
    if (!release) {
        ++joystick_sequence_;
        arbiter_.applyRates(state::SourceId::Joystick, rates, 1, now,
                            now + CONFIG_CTRL_INTENT_LEASE_US);
    }
}

void ControlService::sendRates(const control::ArbitratedSetpoint &setpoint)
{
    const std::uint32_t limit = CONFIG_CTRL_DELIVERY_FAILURE_LIMIT;
    for (std::size_t i = 0; i < 2; ++i) {
        const motion::AxisId id = static_cast<motion::AxisId>(i);
        const float desired = setpoint.have_command ? setpoint.rate[i] : 0.0F;
        const control::RateGate::Decision decision =
            gate_.evaluate(id, desired, arbiter_.generation());
        if (!decision.send) {
            continue;
        }
        const control::Delivery delivery = port_.setRate(id, decision.value);
        if (delivery.accepted) {
            gate_.noteEnqueued(id, decision.value, arbiter_.generation());
            delivery_failures_[i] = 0;
            if (delivery.ticket.valid()) {
                last_stop_ticket_[i] = delivery.ticket.id;
            }
        } else {
            gate_.noteFailed(id);
            if (delivery_failures_[i] < limit) {
                ++delivery_failures_[i];
            }
        }
    }
}

void ControlService::captureStopAttestation(Micros now)
{
    (void)now;
    // Ровно одна логическая отмена разрешения на подтверждённый останов:
    // executeStop() не вызывается повторно для подтверждённого протокола.
    revokeMotionPermission();

    const std::uint64_t request_id = stop_protocol_.requestId();
    if (request_id == 0U) {
        return;
    }
    CalibrationStopAttestation attestation{};
    attestation.valid = true;
    attestation.confirmed = true;
    attestation.request_id = request_id;
    attestation.requested_us = stop_protocol_.requestedUs();
    attestation.confirmed_us = stop_protocol_.confirmedUs();
    attestation.immediate = stop_protocol_.mode() == control::StopMode::Immediate;
    for (std::size_t i = 0; i < 2; ++i) {
        const control::CommandTicket ticket = stop_protocol_.ticket(i);
        attestation.ticket_id[i] = ticket.id;
        attestation.epoch[i] = ticket.epoch;
        attestation.position_step[i] = axis_[i].position_step;
        attestation.polled_us[i] = axis_polled_[i];
        attestation.position_trusted[i] =
            axis_[i].position_trusted && !axis_[i].has_fault;
        attestation.energized[i] = axis_[i].energized;
    }
    attestation.motion_generation = motion_generation_.load();

    if (lockCal(portMAX_DELAY)) {
        attestation_ = attestation;
        unlockCal();
    }
    pending_calibration_stop_.store(0);
}

void ControlService::executeStop(Micros now)
{
    if (!stop_protocol_.active() || stop_protocol_.confirmed()) {
        return;
    }
    const control::StopProtocol::Action action = stop_protocol_.next();
    for (std::size_t i = 0; i < 2; ++i) {
        if (!action.send[i]) {
            continue;
        }
        const motion::AxisId id = static_cast<motion::AxisId>(i);
        const control::Delivery delivery = action.mode == control::StopMode::Immediate
                                               ? port_.stopImmediate(id)
                                               : port_.stopSmooth(id);
        if (delivery.accepted) {
            stop_protocol_.noteSent(id, delivery);
        } else {
            stop_protocol_.noteFailed(id);
            if (delivery_failures_[i] < CONFIG_CTRL_DELIVERY_FAILURE_LIMIT) {
                ++delivery_failures_[i];
            }
        }
    }

    control::StopObservation observations[2]{};
    for (std::size_t i = 0; i < 2; ++i) {
        observations[i].polled_us = axis_polled_[i];
        observations[i].status = axis_[i];
    }
    switch (stop_protocol_.check(observations, now)) {
        case control::StopProtocol::Decision::Confirmed:
            stop_confirmed_flag_ = true;
            captureStopAttestation(now);
            break;
        case control::StopProtocol::Decision::Escalate:
            if (stop_protocol_.escalate(now)) {
                ++escalations_;
            }
            break;
        case control::StopProtocol::Decision::Failed:
            stop_failed_flag_ = true;
            if (stop_timeouts_ < UINT32_MAX) {
                ++stop_timeouts_;
            }
            break;
        case control::StopProtocol::Decision::Pending:
        case control::StopProtocol::Decision::Idle:
            break;
    }
}

void ControlService::driveLoop(Micros now)
{
    // Терминальный исход протокола потреблён автоматом: завершаем жизненный
    // цикл, иначе подтверждённый протокол навсегда заблокирует уставки.
    if (stop_protocol_.active() &&
        (stop_protocol_.confirmed() || stop_failed_flag_) &&
        arming_.phase() != state::ControlPhase::Stopping) {
        stop_protocol_.reset();
        stop_failed_flag_ = false;
        stop_confirmed_flag_ = false;
    }

    const control::ArbitratedSetpoint setpoint = arbiter_.arbitrate(nowUs());

    control::ArmingInputs inputs{};
    inputs.permit = permit_requested_.load() && !flash_window_active_.load();
    inputs.neutral_confirmed =
        conversion_.ok && conversion_.neutral &&
        state::freshness(nowUs(), joystick_obs_.meta.measured_us,
                         CONFIG_CTRL_AXIS_STALE_US) == state::Freshness::Fresh;
    inputs.input_usable = input_usable_;
    inputs.selected_source = arbiter_.owner();
    inputs.calibration_input_ready =
        inputs.selected_source == state::SourceId::Calibration && calibrationInputReady(nowUs());
    inputs.motor_fault = axesHaveFault();
    inputs.axes_ready = axesReady();
    // Потеря готовности уже разрешённой оси: останов пары и защёлка.
    // В фазе Enabling готовности ещё нет по определению — там работает
    // отдельный счётчик отказов доставки Enable.
    inputs.axis_lost = !inputs.axes_ready &&
                       arming_.phase() == state::ControlPhase::Active;
    inputs.enable_failed = enable_failures_[0] >= CONFIG_CTRL_DELIVERY_FAILURE_LIMIT ||
                           enable_failures_[1] >= CONFIG_CTRL_DELIVERY_FAILURE_LIMIT;
    inputs.rate_delivery_failed =
        delivery_failures_[0] >= CONFIG_CTRL_DELIVERY_FAILURE_LIMIT ||
        delivery_failures_[1] >= CONFIG_CTRL_DELIVERY_FAILURE_LIMIT;
    inputs.stop_requested = safety_stop_requested_.exchange(false) ||
                            shutdown_requested_.load();
    inputs.calibration_stop = calibration_stop_requested_.exchange(false);
    inputs.stopped_confirmed = stoppedConfirmed();
    inputs.stop_failed = stop_failed_flag_;
    stop_failed_flag_ = false;  // исход прошлого цикла передан автомату
    inputs.handover_pending = setpoint.handover_pending;
    if (setpoint.handover_pending) {
        control::DenyReason deny = control::DenyReason::None;
        inputs.handover_valid = arbiter_.revalidateClaim(nowUs(), deny) && inputs.permit;
    }

    const control::ArmingDecision decision = arming_.step(inputs);
    setLatch(decision.latch);

    if (decision.revoke_ownership) {
        const state::SourceId previous = arbiter_.owner();
        arbiter_.release(previous, nowUs());
        if (previous == state::SourceId::Calibration) {
            require_neutral_after_calibration_ = true;
            calibration_released_us_ = nowUs();
        }
        // При передаче владения претендент сохраняется до revalidateClaim.
        if (decision.stop_reason != control::StopReason::Handover) {
            arbiter_.cancelHandover();
        }
    }

    // Отзыв разрешения: переход «действовало → отозвано» увеличивает
    // поколение ровно один раз.
    const bool calibration_permission =
        inputs.selected_source == state::SourceId::Calibration &&
        inputs.calibration_input_ready && decision.latch == state::LatchCode::None;
    if (calibration_permission) {
        calibration_permission_active_ = true;
    } else if (calibration_permission_active_) {
        revokeMotionPermission();
        safety_stop_requested_.store(true);
    }

    if (decision.request_stop && !stop_protocol_.active()) {
        control::StopProtocol::Start start{};
        start.reason = decision.stop_reason;
        start.mode = control::StopMode::Smooth;
        start.now_us = nowUs();
        start.baseline_epoch[0] = axis_[0].command_epoch;
        start.baseline_epoch[1] = axis_[1].command_epoch;
        start.request_id = decision.stop_reason == control::StopReason::Calibration
                               ? pending_calibration_stop_.exchange(0)
                               : 0;
        if (decision.stop_reason != control::StopReason::Calibration) {
            pending_calibration_stop_.store(0);
        }
        stop_protocol_.begin(start);
        arming_.noteStopIssued(decision.stop_reason);
        stop_failed_flag_ = false;
        stop_confirmed_flag_ = false;
    }

    if (decision.send_enable) {
        enable_accepted_[0] = false;
        enable_accepted_[1] = false;
        enable_failures_[0] = 0;
        enable_failures_[1] = 0;
        enable_pending_ = true;
    }
    if (arming_.phase() == state::ControlPhase::Enabling) {
        // Повтор только для недоставленного Enable (команда идемпотентна).
        for (std::size_t i = 0; i < 2; ++i) {
            if (enable_accepted_[i]) {
                continue;
            }
            const control::Delivery delivery =
                port_.enable(static_cast<motion::AxisId>(i));
            if (delivery.accepted) {
                enable_accepted_[i] = true;
                enable_failures_[i] = 0;
            } else if (enable_failures_[i] < CONFIG_CTRL_DELIVERY_FAILURE_LIMIT) {
                ++enable_failures_[i];
            }
        }
    } else {
        enable_pending_ = false;
    }

    if (stop_protocol_.active()) {
        executeStop(nowUs());
    } else if (decision.allow_nonzero && !flash_window_active_.load()) {
        sendRates(setpoint);
    }

    // Окно записи NVS открывается только после подтверждённого останова и
    // покоя обеих осей.
    if (flash_window_requested_.load() && !flash_window_active_.load()) {
        const bool at_rest = control::StopProtocol::axisAtRest(axis_[0]) &&
                             control::StopProtocol::axisAtRest(axis_[1]);
        const bool stopped = !stop_protocol_.active() || stop_protocol_.confirmed();
        if (at_rest && stopped) {
            flash_window_active_.store(true);
            flash_window_requested_.store(false);
        } else if (nowUs() > flash_window_requested_us_.load()) {
            flash_window_requested_.store(false);
        }
    }

    if (decision.handover_complete) {
        arbiter_.completeHandover(nowUs());
    } else if (arbiter_.handoverPending() &&
               arming_.phase() != state::ControlPhase::Stopping) {
        // Передача не состоялась: претендент отклонён или протокол прерван.
        arbiter_.cancelHandover();
    }
}

void ControlService::runRecovery()
{
    std::uint32_t request_id = 0;
    Micros deadline = 0;
    (void)xSemaphoreTake(recovery_mutex_, portMAX_DELAY);
    if (recovery_.state == RecoveryState::Pending && !recovery_.execution_busy) {
        request_id = recovery_.request_id;
        deadline = recovery_.deadline_us;
        recovery_.execution_busy = true;
    }
    (void)xSemaphoreGive(recovery_mutex_);
    if (request_id == 0U) {
        return;
    }

    permit_requested_.store(false);
    arbiter_.release(arbiter_.owner(), nowUs());
    arbiter_.cancelHandover();

    bool ok = false;
    esp_err_t code = ESP_FAIL;

    control::StopProtocol::Start start{};
    start.reason = control::StopReason::Acknowledge;
    start.mode = control::StopMode::Smooth;
    start.now_us = nowUs();
    start.baseline_epoch[0] = axis_[0].command_epoch;
    start.baseline_epoch[1] = axis_[1].command_epoch;
    stop_protocol_.begin(start);
    stop_failed_flag_ = false;

    bool interrupted = false;
    while (nowUs() < deadline) {
        // Приоритетные запросы отменяют восстановление между шагами.
        if (shutdown_requested_.load() || safety_stop_requested_.load()) {
            interrupted = true;
            break;
        }
        pollInputs(nowUs());
        executeStop(nowUs());
        if (stop_failed_flag_) {
            code = ESP_ERR_TIMEOUT;
            break;
        }
        if (!stop_protocol_.confirmed()) {
            vTaskDelay(msToTicks(kRecoveryStepMs));
            continue;
        }
        if (!restOrCannotMove(axis_[0]) || !restOrCannotMove(axis_[1])) {
            vTaskDelay(msToTicks(kRecoveryStepMs));
            continue;
        }

        const Micros reset_sent_us = nowUs();
        const control::Delivery faults[2] = {port_.resetFault(motion::AxisId::Azimuth),
                                             port_.resetFault(motion::AxisId::Altitude)};
        bool reset_ok = true;
        while (nowUs() < deadline) {
            if (shutdown_requested_.load() || safety_stop_requested_.load()) {
                interrupted = true;
                break;
            }
            pollInputs(nowUs());
            bool confirmed = true;
            for (std::size_t i = 0; i < 2; ++i) {
                const motion::AxisStatus &status = axis_[i];
                // Совпавший ack доказывает только обработку команды, но не
                // успешный сброс; нужен ещё свежий статус без fault.
                const bool matched = faults[i].accepted && faults[i].ticket.valid() &&
                                     status.command_epoch == faults[i].ticket.epoch &&
                                     status.last_applied_command_id == faults[i].ticket.id;
                const bool disabled = status.state == motion::AxisState::Disabled &&
                                      !status.has_fault && !status.energized;
                const bool fresh = axis_polled_[i] > reset_sent_us;
                if (!(matched && disabled && fresh)) {
                    confirmed = false;
                    break;
                }
            }
            if (confirmed) {
                reset_ok = true;
                break;
            }
            reset_ok = false;
            vTaskDelay(msToTicks(kRecoveryStepMs));
        }
        if (!reset_ok) {
            code = ESP_ERR_TIMEOUT;
            break;
        }
        ok = true;
        break;
    }

    (void)xSemaphoreTake(recovery_mutex_, portMAX_DELAY);
    if (recovery_.state != RecoveryState::Pending ||
        recovery_.request_id != request_id) {
        ++recovery_late_;
    } else if (interrupted || shutdown_requested_.load() ||
               safety_stop_requested_.load() || recovery_cancel_requested_) {
        // Приоритетный запрос, пришедший во время финализации, не даёт
        // зафиксировать успех: защёлка остаётся.
        recovery_.state = RecoveryState::Cancelled;
        recovery_.result = ESP_ERR_INVALID_STATE;
        ++recovery_cancelled_;
    } else if (nowUs() > deadline) {
        recovery_.state = RecoveryState::Cancelled;
        recovery_.result = ESP_ERR_TIMEOUT;
        ++recovery_cancelled_;
    } else if (ok) {
        recovery_.state = RecoveryState::Succeeded;
        recovery_.result = ESP_OK;
        latch_ = state::LatchCode::None;
        arming_.clearLatch();
    } else {
        recovery_.state = RecoveryState::Failed;
        recovery_.result = code;
        ++recovery_failed_;
    }
    recovery_.execution_busy = false;
    recovery_cancel_requested_ = false;
    (void)xSemaphoreGive(recovery_mutex_);

    if (recovery_done_ != nullptr) {
        (void)xSemaphoreGive(recovery_done_);
    }
}

void ControlService::applyCalibrationRequests(Micros now)
{
    // 1. Двухфазные заявки на временное удержание опорной точки.
    for (std::size_t i = 0; i < 2; ++i) {
        HoldRequest request{};
        bool have = false;
        if (lockCal(portMAX_DELAY)) {
            if (hold_request_[i].pending) {
                request = hold_request_[i];
                hold_request_[i] = HoldRequest{};
                have = true;
            }
            unlockCal();
        }
        if (!have) {
            continue;
        }
        const motion::AxisId id = static_cast<motion::AxisId>(i);
        CalibrationHold &hold = calibration_hold_[i];
        if (request.begin) {
            if (!hold.active) {
                // Исходная политика берётся из конфигурации, а не угадывается.
                hold.original = port_.holdPolicy(id);
                hold.restore_pending = false;
                hold.restore_ticket = control::CommandTicket{};
                hold.active = true;
                hold.session = request.session;
                hold.deadline = request.deadline;
                calibration_ready_seen_ = false;
                (void)port_.setHoldPolicy(id, motion::HoldPolicy{true, 0});
            } else if (hold.session != request.session) {
                // Новая сессия поверх старой: сначала восстанавливаем
                // сохранённую исходную политику, иначе она будет потеряна.
                const motion::HoldPolicy original = hold.original;
                (void)port_.setHoldPolicy(id, original);
                hold = CalibrationHold{};
                hold.original = original;
                hold.active = true;
                hold.session = request.session;
                hold.deadline = request.deadline;
                calibration_ready_seen_ = false;
                (void)port_.setHoldPolicy(id, motion::HoldPolicy{true, 0});
            } else if (request.deadline != 0U &&
                       (hold.deadline == 0U || request.deadline < hold.deadline)) {
                hold.deadline = request.deadline;  // не расширяем
            }
        } else if (hold.session == request.session && (hold.active || hold.restore_pending)) {
            // Идемпотентно: восстанавливаем сохранённую исходную политику.
            // Подтверждение — по применённому тикету, а не по приёму в очередь.
            const control::Delivery delivery = port_.setHoldPolicy(id, hold.original);
            if (delivery.accepted && delivery.ticket.valid()) {
                hold.active = false;
                hold.restore_pending = true;
                hold.restore_ticket = delivery.ticket;
            }
        }
    }

    const state::SourceId owner = arbiter_.owner();
    const bool ready = owner == state::SourceId::Calibration && calibrationInputReady(now);
    if (ready) {
        calibration_ready_seen_ = true;
    }
    const bool readiness_lost = calibration_ready_seen_ && !ready;

    // 2. Абсолютный предел удержания и потеря пригодности источника.
    for (std::size_t i = 0; i < 2; ++i) {
        CalibrationHold &hold = calibration_hold_[i];
        if (!hold.active) {
            continue;
        }
        const motion::AxisId id = static_cast<motion::AxisId>(i);
        const bool deadline_hit = hold.deadline != 0U && now >= hold.deadline;
        const bool may_restore =
            deadline_hit ||
            (readiness_lost && (stop_protocol_.confirmed() ||
                                control::StopProtocol::axisAtRest(axis_[i])));
        if (!may_restore) {
            continue;
        }
        const control::Delivery delivery = port_.setHoldPolicy(id, hold.original);
        if (!delivery.accepted || !delivery.ticket.valid()) {
            continue;
        }
        hold.active = false;
        hold.restore_pending = true;
        hold.restore_ticket = delivery.ticket;
        if (deadline_hit) {
            revokeMotionPermission();
            safety_stop_requested_.store(true);
        }
    }

    // 3. Подтверждение восстановления исходной политики: тикет применён.
    for (std::size_t i = 0; i < 2; ++i) {
        CalibrationHold &hold = calibration_hold_[i];
        if (hold.active || !hold.restore_pending) {
            continue;
        }
        if (axis_[i].last_applied_command_id == hold.restore_ticket.id &&
            axis_[i].command_epoch == hold.restore_ticket.epoch) {
            hold = CalibrationHold{};
        }
    }

    if (readiness_lost && calibration_permission_active_) {
        revokeMotionPermission();
        safety_stop_requested_.store(true);
    }
}

void ControlService::cycle()
{
    const Micros now = nowUs();
    ++cycles_;

    pollInputs(now);
    if (!flash_window_active_.load()) {
        applyExternalIntents(now);
        applyJoystick(now);
    }
    arbiter_.expire(now);

    if (arbiter_.consumeOwnerExpired()) {
        permit_requested_.store(false);
        revokeMotionPermission();
        safety_stop_requested_.store(true);
        if (calibration_hold_[0].active || calibration_hold_[1].active) {
            require_neutral_after_calibration_ = true;
            calibration_released_us_ = now;
        }
    }

    applyCalibrationRequests(now);

    // Запрос окна записи NVS: движение запрещается до подтверждённого покоя.
    if (flash_window_requested_.load() && !flash_window_active_.load()) {
        permit_requested_.store(false);
        const state::SourceId previous = arbiter_.owner();
        if (previous != state::SourceId::None) {
            arbiter_.release(previous, now);
            arbiter_.cancelHandover();
            if (previous == state::SourceId::Calibration) {
                require_neutral_after_calibration_ = true;
                calibration_released_us_ = now;
            }
        }
        safety_stop_requested_.store(true);
    }

    if (recovery_mutex_ != nullptr) {
        (void)xSemaphoreTake(recovery_mutex_, portMAX_DELAY);
        const bool recover = recovery_.state == RecoveryState::Pending &&
                             !recovery_.execution_busy;
        (void)xSemaphoreGive(recovery_mutex_);
        if (recover) {
            runRecovery();
        }
    }

    driveLoop(now);
    publishControl(now);
}

void ControlService::publishControl(Micros now)
{
    state::ControlObservation obs{};
    obs.phase = arming_.phase();
    obs.latch = arming_.latch();
    obs.owner = arbiter_.owner();
    obs.owner_generation = arbiter_.generation();
    obs.permit = permit_requested_.load();
    obs.recovery_active = false;
    {
        (void)xSemaphoreTake(recovery_mutex_, portMAX_DELAY);
        obs.recovery_active = recovery_.execution_busy ||
                              recovery_.state == RecoveryState::Pending;
        (void)xSemaphoreGive(recovery_mutex_);
    }
    obs.deny = conversion_.deny;
    for (std::size_t i = 0; i < 2; ++i) {
        const motion::AxisId id = static_cast<motion::AxisId>(i);
        obs.desired_rate[i] = joystick_obs_.neutral ? 0.0F : conversion_.rate[i];
        obs.enqueued_rate[i] = gate_.lastEnqueued(id);
        obs.enqueued_known[i] = gate_.hasEnqueuedNonZero(id) || gate_.pending(id);
        obs.confirmed_rate[i] = axis_[i].rate_steps_s;
        obs.last_ticket_id[i] = last_stop_ticket_[i];
        obs.last_applied_id[i] = axis_[i].last_applied_command_id;
        obs.command_epoch[i] = axis_[i].command_epoch;
        obs.delivery_failures[i] = delivery_failures_[i];
    }
    obs.stop_requested_us = stop_protocol_.requestedUs();
    obs.stop_confirmed = stop_protocol_.confirmed();
    obs.stop_reason = static_cast<std::uint8_t>(stop_protocol_.reason());
    obs.stop_timeouts = stop_timeouts_;
    obs.escalations = escalations_;
    obs.ack_mismatches = ack_mismatches_;
    obs.emergency_slot_busy = motors_.emergencySlotBusy();
    obs.unmatched_emergency_notifications =
        axis_[0].unmatched_emergency_notifications +
        axis_[1].unmatched_emergency_notifications;
    obs.notify_failures = motors_.notifyFailures();
    obs.rejected_intents = rejected_intents_;
    obs.cycle = cycles_;
    obs.meta.polled_us = now;
    hub_.publishControl(obs);
}

void ControlService::finishShutdown()
{
    permit_requested_.store(false);
    safety_stop_requested_.store(false);

    control::StopProtocol::Start start{};
    start.reason = control::StopReason::Shutdown;
    start.mode = control::StopMode::Smooth;
    start.now_us = nowUs();
    for (std::size_t i = 0; i < 2; ++i) {
        const motion::AxisId id = static_cast<motion::AxisId>(i);
        axis_[i] = port_.status(id);
        axis_polled_[i] = nowUs();
        start.baseline_epoch[i] = axis_[i].command_epoch;
    }
    stop_protocol_.begin(start);
    stop_failed_flag_ = false;

    const Micros deadline =
        nowUs() + static_cast<Micros>(CONFIG_CTRL_STOP_CONFIRM_TIMEOUT_US) * 3U;
    while (nowUs() < deadline) {
        pollInputs(nowUs());
        executeStop(nowUs());
        if (stop_protocol_.confirmed()) {
            break;
        }
        vTaskDelay(msToTicks(kRecoveryStepMs));
    }

    if (!stop_protocol_.confirmed()) {
        // Последняя мера: приоритетный останов обеих осей.
        (void)port_.stopImmediate(motion::AxisId::Azimuth);
        (void)port_.stopImmediate(motion::AxisId::Altitude);
        stop_failed_.store(true);
        setLatch(state::LatchCode::StopUnconfirmed);
        ESP_LOGE(kTag, "stop not confirmed before shutdown");
    } else {
        stop_failed_.store(false);
    }
}

}  // namespace control_hal
