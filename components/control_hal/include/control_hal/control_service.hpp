/**
 * @file control_service.hpp
 * @brief Управляющая задача: единственный исполнитель моторных операций.
 *
 * Контракт:
 *  - все команды в MotorService идут через MotorAxisPort и только из задачи;
 *  - внешние вызовы (start/stop/arm/disarm/requestSafetyStop/submitIntent/
 *    acknowledgeFault) не касаются порта, а ставят запросы;
 *  - api_mutex_ сериализует только внешний API; задача берёт его никогда;
 *  - состояние запроса восстановления защищено recovery_mutex_, который
 *    никогда не удерживается при вызове порта, ожидании семафора и
 *    завершении задачи; защёлка снимается только в терминальной финализации.
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "control/arbiter.hpp"
#include "control/arming.hpp"
#include "control/input_grace.hpp"
#include "control/intent_inbox.hpp"
#include "control/joy_mapping.hpp"
#include "control/rate_gate.hpp"
#include "control/stop_protocol.hpp"
#include "control_hal/external_intents.hpp"
#include "control_hal/motor_axis_port.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "input_hal/joystick_service.hpp"
#include "motor_hal/motor_service.hpp"
#include "state/health.hpp"
#include "state/observation.hpp"
#include "state_hal/state_hub.hpp"

namespace control_hal {

/// Доказательство подтверждённого останова для калибровки.
/// Заполняется управляющей задачей при `Decision::Confirmed` и **не**
/// стирается обычным `stop_protocol_.reset()`.
struct CalibrationStopAttestation {
    bool valid = false;
    std::uint64_t request_id = 0;
    std::uint64_t ticket_id[2] = {0, 0};
    std::uint32_t epoch[2] = {0, 0};
    std::int64_t position_step[2] = {0, 0};
    state::Micros requested_us = 0;
    state::Micros confirmed_us = 0;
    state::Micros polled_us[2] = {0, 0};
    bool position_trusted[2] = {false, false};
    bool energized[2] = {false, false};
    bool immediate = false;
    bool confirmed = false;
    std::uint32_t motion_generation = 0;
};

/// Снимок состояния контура для диагностики и будущего UI.
struct ControlStatus {
    state::ControlPhase phase = state::ControlPhase::Disarmed;
    state::LatchCode latch = state::LatchCode::None;
    state::SourceId owner = state::SourceId::None;
    std::uint32_t owner_generation = 0;
    bool permit = false;
    bool recovery_active = false;
    state::DenyReason deny = state::DenyReason::None;

    float desired_rate[2] = {0.0F, 0.0F};
    float enqueued_rate[2] = {0.0F, 0.0F};
    float confirmed_rate[2] = {0.0F, 0.0F};

    bool joystick_valid = false;
    bool joystick_fresh = false;
    bool joystick_neutral = true;
    bool joystick_fault = false;

    std::uint32_t delivery_failures[2] = {0, 0};
    std::uint32_t stop_timeouts = 0;
    std::uint32_t escalations = 0;
    std::uint32_t ack_mismatches = 0;
    std::uint32_t emergency_slot_busy = 0;
    std::uint32_t unmatched_emergency_notifications = 0;
    std::uint32_t notify_failures = 0;
    std::uint32_t rejected_intents = 0;
    std::uint32_t cycles = 0;
};

class ControlService {
public:
    ControlService(motor_hal::MotorService &motors,
                   input_hal::JoystickService &joystick,
                   state_hal::StateHub &hub);
    ~ControlService();

    ControlService(const ControlService &) = delete;
    ControlService &operator=(const ControlService &) = delete;

    /// Разрешён и при сохранённой защёлке: служба поднимается в Disarmed.
    /// Отклоняется, если запись восстановления ещё используется.
    esp_err_t start(const control::JoyMappingConfig &mapping);
    /// Самодостаточный останов: запрет намерений, протокол останова,
    /// эскалация, завершение задачи. Предварительные вызовы не нужны.
    esp_err_t stop();

    esp_err_t arm();
    esp_err_t disarm();
    esp_err_t requestSafetyStop();

    /// Неблокирующая передача намерения будущих Browser и Automation.
    esp_err_t submitIntent(const control::Intent &intent);

    /// Приоритетный запрос останова для калибровки. Не блокирует: задача
    /// запустит StopProtocol и увеличит motion_generation один раз на
    /// логический останов. Повтор с тем же request_id не создаёт операцию.
    esp_err_t requestCalibrationStop(std::uint64_t request_id);

    /// Копия последнего подтверждённого останова; моторы не вызывает.
    CalibrationStopAttestation lastStopAttestation() const;

    /// Временное бессрочное удержание оси на время сессии. Исходная политика
    /// сохраняется и восстанавливается `endCalibrationHold`. Применяется
    /// управляющей задачей; возврат — только факт постановки запроса.
    esp_err_t beginCalibrationHold(motion::AxisId axis, std::uint32_t session,
                                   state::Micros absolute_deadline_us);
    esp_err_t endCalibrationHold(motion::AxisId axis, std::uint32_t session);
    bool calibrationHoldActive(motion::AxisId axis, std::uint32_t session) const;
    /// true только когда удержания нет и восстановление политики применено.
    bool calibrationHoldCleared(motion::AxisId axis) const;

    /// Текущая (возможно временная) политика удержания оси.
    motion::HoldPolicy holdPolicy(motion::AxisId axis) const;

    /// Публикация пригодности калибровочного входа с lease. Истёкший lease
    /// считается ложью; исправный джойстик отказ не маскирует.
    void setCalibrationInputReady(bool ready, std::uint32_t session,
                                  state::Micros valid_until_us);

    /// Текущее поколение разрешения движения.
    std::uint32_t motionGeneration() const;

    /// Окно записи NVS: запрещает движение до `endFlashWindow`. Запрос
    /// ставится в очередь; вызывающий ждёт `flashWindowActive()` и по
    /// таймауту считает окно не открытым.
    esp_err_t beginFlashWindow(state::Micros timeout_us);
    void endFlashWindow();
    bool flashWindowActive() const;

    /// Текущая пригодность калибровочного входа с учётом lease.
    bool calibrationInputReady() const;

    /// Асинхронный запрос; 0 — взять CONFIG_CTRL_ACK_TIMEOUT_MS.
    esp_err_t acknowledgeFault(state::Micros timeout_us = 0);

    ControlStatus status() const;

private:
    enum class Lifecycle : std::uint8_t { Stopped, Running, Stopping };
    enum class RecoveryState : std::uint8_t {
        Idle,
        Pending,
        Succeeded,
        Cancelled,
        Failed
    };

    /// Защищено recovery_mutex_.
    struct RecoveryRecord {
        RecoveryState state = RecoveryState::Idle;
        std::uint32_t request_id = 0;
        state::Micros deadline_us = 0;
        bool execution_busy = false;
        bool result_unread = false;
        esp_err_t result = ESP_OK;
    };

    /// Двухфазная заявка на временное удержание (применяется задачей).
    struct HoldRequest {
        bool pending = false;
        bool begin = true;
        std::uint32_t session = 0;
        state::Micros deadline = 0;
    };

    /// Защищено cal_mutex_.
    struct CalibrationHold {
        bool active = false;
        /// Восстановление исходной политики отправлено, но ещё не применено.
        bool restore_pending = false;
        std::uint32_t session = 0;
        motion::HoldPolicy original{};
        state::Micros deadline = 0;
        control::CommandTicket restore_ticket{};
    };

    static void taskEntry(void *arg);
    void run();
    void cycle();
    void finishShutdown();

    void applyCalibrationRequests(state::Micros now_us);
    void captureStopAttestation(state::Micros now_us);
    void revokeMotionPermission() noexcept;
    bool calibrationInputReady(state::Micros now_us) const noexcept;
    bool lockCal(TickType_t wait) const;
    void unlockCal() const;

    void pollInputs(state::Micros now_us);
    void applyExternalIntents(state::Micros now_us);
    void applyJoystick(state::Micros now_us);
    void driveLoop(state::Micros now_us);
    void executeStop(state::Micros now_us);
    void sendRates(const control::ArbitratedSetpoint &setpoint);
    void runRecovery();
    void publishControl(state::Micros now_us);

    state::Micros nowUs() const noexcept;
    static std::size_t indexOf(motion::AxisId id) noexcept;
    bool lockApi(TickType_t wait) const;
    void unlockApi() const;
    void setLatch(state::LatchCode latch) noexcept;
    state::LatchCode latch() const noexcept;
    /// Отмена восстановления под recovery_mutex_: только пока запрос Pending.
    void markRecoveryCancelledIfPending() noexcept;
    bool axesHaveFault() const noexcept;
    bool axesReady() const noexcept;
    bool stoppedConfirmed() const noexcept;

    motor_hal::MotorService &motors_;
    input_hal::JoystickService &joystick_;
    state_hal::StateHub &hub_;
    MotorAxisPort port_;

    control::JoyMappingConfig mapping_{};
    control::InputGracePolicy grace_{};
    control::RateGate gate_{};
    control::StopProtocol stop_protocol_{};
    control::ArmingMachine arming_{};
    control::Arbiter arbiter_{};
    control::IntentInbox inbox_{};
    ExternalIntents external_{};

    TaskHandle_t task_ = nullptr;
    TickType_t period_ticks_ = 1;
    Lifecycle lifecycle_ = Lifecycle::Stopped;

    StaticSemaphore_t api_mutex_storage_{};
    SemaphoreHandle_t api_mutex_ = nullptr;
    StaticSemaphore_t cal_mutex_storage_{};
    SemaphoreHandle_t cal_mutex_ = nullptr;
    StaticSemaphore_t recovery_mutex_storage_{};
    SemaphoreHandle_t recovery_mutex_ = nullptr;
    StaticSemaphore_t exit_sem_storage_{};
    SemaphoreHandle_t exit_sem_ = nullptr;
    StaticSemaphore_t recovery_done_storage_{};
    SemaphoreHandle_t recovery_done_ = nullptr;

    std::atomic<bool> permit_requested_{false};
    std::atomic<bool> safety_stop_requested_{false};
    /// Останов калибровки: не отзывает владение и разрешение.
    std::atomic<bool> calibration_stop_requested_{false};
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> stop_failed_{false};

    // Калибровка: заявки и результаты под cal_mutex_.
    HoldRequest hold_request_[2]{};
    CalibrationHold calibration_hold_[2]{};
    CalibrationStopAttestation attestation_{};
    std::atomic<bool> calibration_ready_{false};
    std::atomic<std::uint32_t> calibration_ready_session_{0};
    std::atomic<state::Micros> calibration_ready_until_{0};
    std::atomic<std::uint32_t> motion_generation_{0};
    std::atomic<std::uint64_t> pending_calibration_stop_{0};
    std::atomic<bool> flash_window_requested_{false};
    std::atomic<state::Micros> flash_window_requested_us_{0};
    std::atomic<bool> flash_window_active_{false};

    RecoveryRecord recovery_{};
    /// Защищён recovery_mutex_: согласует отмену с терминальным переходом.
    bool recovery_cancel_requested_ = false;
    state::LatchCode latch_ = state::LatchCode::None;
    std::uint32_t next_recovery_id_ = 1;

    // Пишет и читает только задача.
    state::JoystickObservation joystick_obs_{};
    control::JoyConversion conversion_{};
    bool input_usable_ = false;
    bool enable_accepted_[2] = {true, true};
    motion::AxisStatus axis_[2]{};
    state::Micros axis_polled_[2] = {0, 0};
    std::uint32_t axis_read_failures_[2] = {0, 0};
    std::uint32_t baseline_epoch_[2] = {0, 0};
    std::uint32_t delivery_failures_[2] = {0, 0};
    std::uint32_t enable_failures_[2] = {0, 0};
    std::uint32_t stop_timeouts_ = 0;
    std::uint32_t escalations_ = 0;
    std::uint32_t ack_mismatches_ = 0;
    std::uint32_t rejected_intents_ = 0;
    std::uint64_t cycles_ = 0;
    std::uint32_t joystick_sequence_ = 0;
    std::uint32_t recovery_timeouts_ = 0;
    std::uint32_t recovery_cancelled_ = 0;
    std::uint32_t recovery_failed_ = 0;
    std::uint32_t recovery_late_ = 0;
    bool stop_failed_flag_ = false;
    bool stop_confirmed_flag_ = false;
    bool enable_pending_ = false;
    bool neutral_tracking_ = false;
    state::Micros neutral_since_ = 0;
    std::uint64_t last_stop_ticket_[2] = {0, 0};

    // Калибровка: пишет и читает только задача.
    bool calibration_permission_active_ = false;
    bool calibration_ready_seen_ = false;
    bool require_neutral_after_calibration_ = false;
    state::Micros calibration_released_us_ = 0;
};

}  // namespace control_hal
