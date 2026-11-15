/**
 * @file axis_planner.hpp
 * @brief Двухфазный планировщик одной оси: профиль, состояние, позиция.
 *
 * Контракт исполнения:
 *  - prepareNextStep() только готовит событие и запоминает его;
 *  - commitPending() подтверждает событие: для is_step = true сдвигает
 *    позицию, для паузы — только завершает её;
 *  - cancelPendingStep() отменяет подготовленное событие без побочных
 *    эффектов.
 *
 * Позиция меняется только на commitPending(), то есть после фактического
 * переключения фаз исполнителем.
 */

#pragma once

#include <cstdint>

#include "motion/axis_command.hpp"
#include "motion/axis_config.hpp"

namespace motion {

/// Событие, подготовленное планировщиком.
struct StepPlan {
    Direction direction = Direction::Forward;
    std::uint32_t interval_us = 0;  ///< мкс до события; HAL переводит в тики
    bool is_step = true;            ///< false = пауза реверса, фазы не меняются
};

/// Снимок состояния оси.
struct AxisStatus {
    AxisState state = AxisState::Disabled;
    std::int64_t position_step = 0;  ///< полушаги, меняется только на commitPending
    float rate_steps_s = 0.0F;
    float target_rate_steps_s = 0.0F;
    bool energized = false;      ///< заполняется исполнителем
    bool homed = false;          ///< до верхнего слоя калибровки всегда false
    bool position_trusted = true;
    /// Заполняются исполнителем, а не планировщиком: подтверждение команд.
    std::uint64_t last_applied_command_id = 0;
    std::uint32_t command_epoch = 0;
    std::uint32_t unmatched_emergency_notifications = 0;
    std::uint64_t late_corrections = 0;
    std::uint64_t late_replans = 0;
    std::uint64_t stale_events = 0;
    std::uint64_t rejected_commands = 0;
    bool has_fault = false;                       ///< причина Fault сохранена
    FaultReason fault_reason = FaultReason::InvalidState;
};

class AxisPlanner {
public:
    bool configure(const AxisConfig &config) noexcept;
    const AxisConfig &config() const noexcept { return config_; }

    /// Применяет команду. false = команда отклонена.
    bool apply(const AxisCommand &cmd) noexcept;

    bool hasPending() const noexcept { return pending_.valid; }

    /// Есть ли намерение двигаться (или незавершённая пауза реверса).
    bool wantsStep() const noexcept;

    bool prepareNextStep(StepPlan &out) noexcept;
    void commitPending() noexcept;
    void cancelPendingStep() noexcept;

    void forceReleased(ReleaseReason reason) noexcept;
    void enterFault(FaultReason reason) noexcept;
    void resetFault() noexcept;

    std::uint32_t planKey() const noexcept;

    void noteLateCorrection() noexcept { ++status_.late_corrections; }
    void noteLateReplan() noexcept { ++status_.late_replans; }
    void noteStaleEvent() noexcept { ++status_.stale_events; }

    const AxisStatus &status() const noexcept { return status_; }

private:
    enum class Phase : std::uint8_t { Idle, Move, Pause };

    struct Pending {
        bool valid = false;
        bool is_pause = false;
        bool reverse_after = false;
        Direction direction = Direction::Forward;
        std::uint32_t interval_us = 0;
        float rate_before = 0.0F;
        float rate_after = 0.0F;
    };

    void finishMotion() noexcept;
    void updateState(float rate_before, float rate_after) noexcept;
    void updateStatus() noexcept;
    bool reject() noexcept;

    AxisConfig config_{};
    bool configured_ = false;
    bool enabled_ = false;
    bool fault_latched_ = false;

    AxisState state_ = AxisState::Disabled;
    Phase phase_ = Phase::Idle;

    bool position_mode_ = false;
    std::int64_t target_position_ = 0;
    std::int64_t position_ = 0;

    float v_ = 0.0F;                 ///< текущая скорость на границе, >= 0
    Direction dir_ = Direction::Forward;
    Direction pause_dir_ = Direction::Forward;
    float target_rate_ = 0.0F;       ///< знаковая уставка, полушагов/с

    bool position_trusted_ = true;
    std::uint32_t intent_rev_ = 0;
    Pending pending_{};
    AxisStatus status_{};
};

}  // namespace motion
