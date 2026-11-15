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

    static void taskEntry(void *arg);
    void run();
    void cycle();
    void finishShutdown();

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
    StaticSemaphore_t recovery_mutex_storage_{};
    SemaphoreHandle_t recovery_mutex_ = nullptr;
    StaticSemaphore_t exit_sem_storage_{};
    SemaphoreHandle_t exit_sem_ = nullptr;
    StaticSemaphore_t recovery_done_storage_{};
    SemaphoreHandle_t recovery_done_ = nullptr;

    std::atomic<bool> permit_requested_{false};
    std::atomic<bool> safety_stop_requested_{false};
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> stop_failed_{false};

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
};

}  // namespace control_hal
