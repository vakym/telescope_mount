/**
 * @file calibration_service.hpp
 * @brief Сервис калибровки: сессия, удержание, движение, сохранение.
 *
 * Это единственный владелец calibration::Session и постоянных коэффициентов.
 * Моторные команды идут только через ControlPort (а значит, через
 * ControlService). HTTP-слой вызывает методы сервиса, но не моторы.
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "calibration/port.hpp"
#include "calibration/session.hpp"
#include "calibration_hal/control_port.hpp"
#include "calibration_hal/nvs_store.hpp"
#include "control_hal/control_service.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "state_hal/state_hub.hpp"

namespace calibration_hal {

/// Запрос на создание сессии.
struct SessionRequest {
    motion::AxisId axis = motion::AxisId::Azimuth;
    motion::Direction direction = motion::Direction::Forward;
    std::uint32_t revolutions = 1;
    std::uint32_t repeats = 2;
};

/// Итог команды сервиса. Коды стабильны для HTTP-контракта.
enum class CommandOutcome : std::uint8_t {
    Ok,
    NotReady,
    InvalidState,
    InvalidArgument,
    Busy,
    StaleSession,
    /// Токен не совпадает с владельцем сессии (наблюдатель не управляет).
    Forbidden,
};

/// Статус постоянного коэффициента оси.
struct AxisScaleStatus {
    calibration::AxisScale scale{};
    std::uint32_t nominal_half_steps_per_revolution = 0;
};

/// Полный снимок для веб-слоя.
struct CalibrationStatus {
    calibration::SessionState state = calibration::SessionState::Inactive;
    calibration::SessionError error = calibration::SessionError::None;
    calibration::InvalidReason invalid_reason = calibration::InvalidReason::Ok;
    std::uint32_t session_id = 0;
    std::uint32_t revision = 0;
    std::uint32_t motion_generation = 0;
    motion::AxisId axis = motion::AxisId::Azimuth;
    motion::Direction direction = motion::Direction::Forward;
    std::uint32_t revolutions = 0;
    std::uint32_t repeats = 0;
    std::uint32_t measurement_count = 0;
    calibration::Measurement measurements[calibration::kMaxMeasurements]{};
    calibration::CombinedResult result{};
    bool mark_recheck_required = false;
    bool hold_active = false;
    bool input_ready = false;
    bool flash_window_active = false;
    std::uint32_t lease_remaining_ms = 0;
    AxisScaleStatus axes[2]{};
};

class CalibrationService {
public:
    CalibrationService(control_hal::ControlService &control, state_hal::StateHub &hub);
    ~CalibrationService();

    CalibrationService(const CalibrationService &) = delete;
    CalibrationService &operator=(const CalibrationService &) = delete;

    /// Загрузка NVS и запуск задачи. Повторный вызов — ошибка.
    esp_err_t start();

    CalibrationStatus status() const;
    AxisScaleStatus axisScale(motion::AxisId axis) const;

    // Команды сессии. session_id и token обязательны: токен выдаётся один раз
    // при создании и не публикуется в общем статусе.
    CommandOutcome createSession(const SessionRequest &request, std::uint32_t &session_id,
                                 std::uint32_t &token);
    CommandOutcome closeSession(std::uint32_t session_id, std::uint32_t token);
    CommandOutcome move(std::uint32_t session_id, std::uint32_t token,
                        motion::Direction direction, float rate,
                        std::uint32_t expected_motion_generation);
    CommandOutcome stop(std::uint32_t session_id, std::uint32_t token);
    CommandOutcome confirmStart(std::uint32_t session_id, std::uint32_t token);
    CommandOutcome confirmEnd(std::uint32_t session_id, std::uint32_t token);
    CommandOutcome resume(std::uint32_t session_id, std::uint32_t token);
    CommandOutcome abandon(std::uint32_t session_id, std::uint32_t token);
    CommandOutcome deleteMeasurement(std::uint32_t session_id, std::uint32_t token,
                                     std::size_t index);
    CommandOutcome addRepeat(std::uint32_t session_id, std::uint32_t token);
    CommandOutcome apply(std::uint32_t session_id, std::uint32_t token,
                         bool acknowledge_low_repeatability);
    /// Сброс необратим: требует явного подтверждения в теле.
    CommandOutcome resetAxis(motion::AxisId axis, bool confirmed);

private:
    static void taskEntry(void *arg);
    void run();
    void step();
    bool lock(TickType_t wait = portMAX_DELAY) const;
    void unlock() const;

    bool tokenValid(std::uint32_t token) const noexcept;
    void finishSessionLocked(calibration::Micros now);
    bool liveReferenceValid(const calibration::Snapshot &snapshot) const noexcept;
    void validateMeasurement(calibration::Micros now, const calibration::Snapshot &snapshot);
    void stepPreparing(calibration::Micros now, const calibration::Snapshot &snapshot);
    void stepActive(calibration::Micros now, const calibration::Snapshot &snapshot);
    void refreshReadiness(calibration::Micros now);
    void driveMove(calibration::Micros now, const calibration::Snapshot &snapshot);
    void superviseStop(calibration::Micros now, const calibration::Snapshot &snapshot);
    void superviseTimeouts(calibration::Micros now, const calibration::Snapshot &snapshot);
    void releaseSession(calibration::Micros now);
    /// 0 — ошибка записи, 1 — результат неизвестен, 2 — проверено.
    int persist(const calibration::CombinedResult &result);
    bool persistNominal(motion::AxisId axis);
    void refreshScale(motion::AxisId axis);
    void failSession(calibration::SessionError error);
    calibration::CalibrationParams makeParams() const;
    bool requestStopInternal(calibration::Micros now);

    control_hal::ControlService &control_;
    ControlPort port_;
    NvsStore store_;

    mutable StaticSemaphore_t mutex_storage_{};
    SemaphoreHandle_t mutex_ = nullptr;

    TaskHandle_t task_ = nullptr;
    std::atomic<bool> running_{false};

    calibration::Session session_;
    calibration::CalibrationParams params_{};
    AxisScaleStatus scales_[2]{};

    std::uint32_t session_id_ = 0;
    std::uint32_t session_token_ = 0;
    std::uint32_t next_session_id_ = 1;
    std::uint32_t token_counter_ = 0;
    std::uint64_t next_stop_request_ = 1;

    bool ownership_claimed_ = false;
    calibration::Micros claim_retry_us_ = 0;
    bool hold_started_ = false;
    calibration::Micros readiness_refresh_us_ = 0;
    calibration::Micros hold_deadline_us_ = 0;

    bool move_active_ = false;
    float move_rate_ = 0.0F;
    motion::Direction move_direction_ = motion::Direction::Forward;
    calibration::Micros move_until_us_ = 0;
    calibration::Micros measure_started_us_ = 0;

    std::uint32_t reference_epoch_ = 0;
    bool stop_pending_ = false;
    std::uint64_t stop_request_id_ = 0;
    calibration::Micros stop_requested_us_ = 0;

    calibration::SessionState last_state_ = calibration::SessionState::Inactive;
    calibration::Micros state_since_us_ = 0;
    calibration::Micros last_activity_us_ = 0;
    calibration::Micros last_ready_us_ = 0;
    bool released_ = false;
};

}  // namespace calibration_hal
