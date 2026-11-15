/**
 * @file axis_executor.hpp
 * @brief Событийный исполнитель одной оси: ожидание, шаг, останов, удержание.
 *
 * Класс не зависит от FreeRTOS и ESP-IDF. Обвязка (AxisMotor) только
 * доставляет биты уведомления и значение времени, все решения принимает
 * исполнитель.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "motion/axis_planner.hpp"
#include "motion/deadline_policy.hpp"
#include "motion/executor_io.hpp"
#include "motion/types.hpp"

namespace motion {

/// Биты уведомления задачи оси.
enum EventBits : std::uint32_t {
    TimerEvent = 1U << 0,
    CommandPending = 1U << 1,
    EmergencyStop = 1U << 2,
    TaskExit = 1U << 3,  ///< используется обвязкой для согласованного shutdown
};

class AxisExecutor {
public:
    struct Params {
        std::uint32_t watchdog_us = 5000;   ///< запас ожидания события таймера
        std::size_t command_budget = 8;     ///< команд за итерацию
        /// Эпоха запуска оси: публикуется в AxisStatus и входит в тикет.
        std::uint32_t command_epoch = 1;
    };

    AxisExecutor(AxisPlanner &planner,
                 AlarmSink &alarm,
                 CoilSink &coil,
                 CommandSource &source,
                 Params params) noexcept;

    /// Обрабатывает одно пробуждение по набору битов. bits == 0 — таймаут.
    void onWakeup(std::uint32_t bits) noexcept;

    /// Сколько ждать следующего события; UINT32_MAX — бесконечно.
    std::uint32_t nextWaitUs() const noexcept;

    /// Остались ли команды, не обработанные из-за бюджета.
    bool hasPendingCommands() const noexcept { return !source_.empty(); }

    AxisStatus status() const noexcept;

    bool holdsTimer() const noexcept { return hold_timer_active_; }

private:
    void handleEmergency(std::uint32_t bits) noexcept;
    void handleCommands() noexcept;
    void handleTimer() noexcept;
    void handleMissed(std::uint64_t now, FaultReason reason) noexcept;
    /// anchor_us — момент, от которого отсчитывается следующее событие:
    /// дедлайн исполненного шага для продолжения движения или now для старта.
    void armNext(std::uint64_t anchor_us) noexcept;
    void startHoldOrRelease() noexcept;
    /// Приводит удержание в соответствие с текущей политикой оси.
    void syncHoldState() noexcept;
    void checkHoldExpiry(std::uint64_t now) noexcept;
    void publish() noexcept;

    AxisPlanner &planner_;
    AlarmSink &alarm_;
    CoilSink &coil_;
    CommandSource &source_;
    Params params_;

    std::uint64_t last_applied_id_ = 0;
    std::uint32_t unmatched_emergency_notifications_ = 0;
    bool last_switch_known_ = false;
    std::uint64_t last_switch_us_ = 0;
    std::uint64_t pending_deadline_us_ = 0;
    std::uint32_t pending_interval_us_ = 0;

    bool hold_timer_active_ = false;
    std::uint64_t hold_deadline_us_ = 0;

    AxisStatus status_{};
};

}  // namespace motion
