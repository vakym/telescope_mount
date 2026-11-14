/**
 * @file axis_executor.cpp
 * @brief Реализация событийного цикла оси.
 *
 * Порядок обработки одного пробуждения:
 *  1. EmergencyStop — приоритетно, без выдачи шага;
 *  2. команды с ограниченным бюджетом, без потери уже полученного TimerEvent;
 *  3. событие таймера — не более одного переключения фаз;
 *  4. проверка истечения удержания независимо от причины пробуждения;
 *  5. публикация статуса.
 */

#include "motion/axis_executor.hpp"

#include <algorithm>
#include <limits>

#include "motion/axis_config.hpp"

namespace motion {
namespace {

/// Насыщающее сложение дедлайнов.
constexpr std::uint64_t satAdd(std::uint64_t a, std::uint64_t b) noexcept
{
    return (a > std::numeric_limits<std::uint64_t>::max() - b)
               ? std::numeric_limits<std::uint64_t>::max()
               : a + b;
}

bool sameHoldPolicy(const motion::HoldPolicy &a, const motion::HoldPolicy &b) noexcept
{
    return a.hold_enabled == b.hold_enabled && a.hold_timeout_ms == b.hold_timeout_ms;
}

}  // namespace

AxisExecutor::AxisExecutor(AxisPlanner &planner,
                           AlarmSink &alarm,
                           CoilSink &coil,
                           CommandSource &source,
                           Params params) noexcept
    : planner_(planner), alarm_(alarm), coil_(coil), source_(source), params_(params)
{
    publish();
}

void AxisExecutor::onWakeup(std::uint32_t bits) noexcept
{
    const std::uint64_t now = alarm_.nowUs();

    if ((bits & EmergencyStop) != 0U) {
        handleEmergency();
        publish();
        return;
    }

    bool commands_applied = false;
    if ((bits & CommandPending) != 0U) {
        handleCommands();
        commands_applied = true;
    }

    // Уже полученное TimerEvent не теряется из-за команд.
    if ((bits & TimerEvent) != 0U) {
        handleTimer();
    } else if (!commands_applied && planner_.hasPending()) {
        // Истечение ожидания FreeRTOS само по себе не доказывает, что прошёл
        // аппаратный дедлайн: watchdog проверяется по абсолютному времени.
        const std::uint64_t watchdog_deadline = pending_deadline_us_ + params_.watchdog_us;
        if (now >= watchdog_deadline) {
            handleMissed(now, FaultReason::WatchdogTimeout);
        }
    }

    checkHoldExpiry(now);
    publish();
}

void AxisExecutor::handleEmergency() noexcept
{
    alarm_.disarm();
    pending_deadline_us_ = 0;
    pending_interval_us_ = 0;
    hold_timer_active_ = false;
    source_.clear();
    planner_.forceReleased(ReleaseReason::EmergencyStop);
    coil_.release();
}

void AxisExecutor::handleCommands() noexcept
{
    AxisCommand cmd{};
    std::size_t processed = 0;
    bool plan_changed = false;
    const bool had_pending = planner_.hasPending();

    while (processed < params_.command_budget && source_.take(&cmd, 1U) == 1U) {
        const std::uint32_t key_before = planner_.planKey();
        const motion::HoldPolicy policy_before = planner_.config().hold;
        const AxisState state_before = planner_.status().state;
        const bool accepted = planner_.apply(cmd);
        if (!accepted) {
            processed += 1U;
            continue;  // счётчик отказов ведёт планировщик
        }
        if (planner_.planKey() != key_before) {
            plan_changed = true;
        }

        switch (cmd.kind) {
        case CommandKind::Disable:
        case CommandKind::StopImmediate:
        case CommandKind::Release:
        case CommandKind::ResetFault:
            coil_.release();
            hold_timer_active_ = false;
            break;
        case CommandKind::Energize:
            coil_.energize();
            break;
        default:
            break;
        }

        // Удержание пересчитывается только при фактическом изменении политики
        // или первом переходе в Holding. Повторные идентичные команды не
        // продлевают таймаут.
        if (cmd.kind == CommandKind::SetHoldPolicy &&
            !sameHoldPolicy(policy_before, planner_.config().hold)) {
            syncHoldState();
        }
        if (cmd.kind == CommandKind::Energize && state_before != AxisState::Holding) {
            syncHoldState();
        }
        processed += 1U;
    }

    const AxisState state = planner_.status().state;
    if (state == AxisState::Disabled || state == AxisState::Fault) {
        alarm_.disarm();
        pending_deadline_us_ = 0;
        pending_interval_us_ = 0;
        hold_timer_active_ = false;
    }

    if (plan_changed && (had_pending || planner_.hasPending())) {
        // Команда отменила или изменила подготовленное событие.
        alarm_.disarm();
        planner_.cancelPendingStep();
        pending_deadline_us_ = 0;
        pending_interval_us_ = 0;
    }

    if (state == AxisState::Accelerating || state == AxisState::Cruising ||
        state == AxisState::Decelerating) {
        hold_timer_active_ = false;  // принятый переход к движению отменяет удержание
    }

    if (!planner_.hasPending()) {
        // Новое движение: следующий дедлайн отсчитывается от текущего момента.
        armNext(alarm_.nowUs());
    }
}

void AxisExecutor::handleTimer() noexcept
{
    if (!planner_.hasPending()) {
        planner_.noteStaleEvent();
        return;
    }

    const std::uint64_t now = alarm_.nowUs();

    const AxisConfig &cfg = planner_.config();
    const StepDue due = classifyWakeup(now, pending_deadline_us_, true, cfg.max_step_lateness_us);

    switch (due) {
    case StepDue::IgnoreStale:
        planner_.noteStaleEvent();
        break;

    case StepDue::IgnoreSpurious:
        // Событие пришло раньше срока: шаг не исполняется, alarm остаётся
        // на прежнем дедлайне.
        if (pending_deadline_us_ > now) {
            if (!alarm_.armAt(pending_deadline_us_)) {
                alarm_.disarm();
                planner_.enterFault(FaultReason::ArmFailure);
                coil_.release();
                pending_deadline_us_ = 0;
                pending_interval_us_ = 0;
            }
        } else {
            planner_.noteStaleEvent();
        }
        break;

    case StepDue::MissedTooFar:
        handleMissed(now, FaultReason::DeadlineMiss);
        break;

    case StepDue::ExecuteNow: {
        StepPlan plan{};
        if (!planner_.prepareNextStep(plan)) {
            pending_deadline_us_ = 0;
            pending_interval_us_ = 0;
            break;
        }

        const std::uint64_t executed_deadline = pending_deadline_us_;

        if (plan.is_step) {
            coil_.advance(plan.direction);
            last_switch_us_ = alarm_.nowUs();
            last_switch_known_ = true;
        }
        planner_.commitPending();
        pending_deadline_us_ = 0;
        pending_interval_us_ = 0;

        // Абсолютное расписание: следующий срок считается от дедлайна
        // исполненного события, а не от момента пробуждения задачи.
        armNext(executed_deadline);
        if (!planner_.hasPending()) {
            startHoldOrRelease();
        }
        break;
    }
    }
}

void AxisExecutor::handleMissed(std::uint64_t now, FaultReason reason) noexcept
{
    const AxisConfig &cfg = planner_.config();
    const std::uint64_t last_switch = last_switch_known_ ? last_switch_us_ : now;

    const DeadlineDecision decision = planNextStep(now,
                                                   pending_deadline_us_,
                                                   last_switch,
                                                   pending_interval_us_,
                                                   cfg.min_phase_interval_us,
                                                   cfg.min_alarm_lead_us,
                                                   cfg.late_tolerance_us,
                                                   cfg.deadline_miss);

    if (decision.action == DeadlineAction::Fault) {
        alarm_.disarm();
        planner_.enterFault(reason);
        coil_.release();
        pending_deadline_us_ = 0;
        pending_interval_us_ = 0;
        return;
    }

    // Ожидающий шаг переносится без commitPending: позиция не меняется.
    if (decision.count_correction) {
        planner_.noteLateCorrection();
    }
    if (decision.count_replan) {
        planner_.noteLateReplan();
    }
    pending_deadline_us_ = decision.deadline_us;
    if (!alarm_.armAt(pending_deadline_us_)) {
        alarm_.disarm();
        planner_.enterFault(FaultReason::ArmFailure);
        coil_.release();
        pending_deadline_us_ = 0;
        pending_interval_us_ = 0;
    }
}

void AxisExecutor::armNext(std::uint64_t anchor_us) noexcept
{
    if (!planner_.wantsStep()) {
        return;
    }

    StepPlan plan{};
    if (!planner_.prepareNextStep(plan)) {
        return;
    }
    if (plan.interval_us == 0U) {
        planner_.cancelPendingStep();
        planner_.enterFault(FaultReason::InvalidState);
        coil_.release();
        return;
    }

    const AxisConfig &cfg = planner_.config();
    const std::uint64_t now = alarm_.nowUs();
    const std::uint64_t last_switch = last_switch_known_ ? last_switch_us_ : now;
    const std::uint64_t desired_next = satAdd(anchor_us, plan.interval_us);

    const DeadlineDecision decision = planNextStep(now,
                                                   desired_next,
                                                   last_switch,
                                                   plan.interval_us,
                                                   cfg.min_phase_interval_us,
                                                   cfg.min_alarm_lead_us,
                                                   cfg.late_tolerance_us,
                                                   cfg.deadline_miss);

    if (decision.count_correction) {
        planner_.noteLateCorrection();
    }
    if (decision.count_replan) {
        planner_.noteLateReplan();
    }

    if (decision.action == DeadlineAction::Fault) {
        planner_.enterFault(FaultReason::DeadlineMiss);
        coil_.release();
        return;
    }

    pending_deadline_us_ = decision.deadline_us;
    pending_interval_us_ = plan.interval_us;

    if (!alarm_.armAt(pending_deadline_us_)) {
        alarm_.disarm();
        planner_.enterFault(FaultReason::ArmFailure);
        coil_.release();
        pending_deadline_us_ = 0;
        pending_interval_us_ = 0;
        return;
    }

    // Принятое возобновление движения снимает отсчёт удержания.
    hold_timer_active_ = false;
}

void AxisExecutor::startHoldOrRelease() noexcept
{
    const AxisConfig &cfg = planner_.config();
    const AxisState state = planner_.status().state;

    if (state == AxisState::Holding) {
        if (!coil_.isEnergized()) {
            coil_.energize();
        }
        if (cfg.hold.hold_timeout_ms > 0U) {
            hold_timer_active_ = true;
            hold_deadline_us_ =
                alarm_.nowUs() + static_cast<std::uint64_t>(cfg.hold.hold_timeout_ms) * 1000U;
        } else {
            hold_timer_active_ = false;  // бессрочное удержание до команды
        }
        return;
    }

    hold_timer_active_ = false;
    if (state == AxisState::Released || state == AxisState::Idle) {
        coil_.release();
    }
}

void AxisExecutor::syncHoldState() noexcept
{
    const AxisConfig &cfg = planner_.config();
    const AxisState state = planner_.status().state;

    if (state == AxisState::Holding) {
        if (!cfg.hold.hold_enabled) {
            coil_.release();
            planner_.forceReleased(ReleaseReason::Explicit);
            hold_timer_active_ = false;
            return;
        }
        if (!coil_.isEnergized()) {
            coil_.energize();
        }
        if (cfg.hold.hold_timeout_ms == 0U) {
            hold_timer_active_ = false;  // бессрочное удержание
        } else {
            // Политика изменилась: срок отсчитывается от текущего момента
            // один раз, а не продлевается каждой командой.
            hold_timer_active_ = true;
            hold_deadline_us_ =
                alarm_.nowUs() + static_cast<std::uint64_t>(cfg.hold.hold_timeout_ms) * 1000U;
        }
        return;
    }

    // Стоящая обесточенная ось не начинает удержание по одной лишь смене
    // политики: фазы подаются только командой Energize или после движения.
    hold_timer_active_ = false;
}

void AxisExecutor::checkHoldExpiry(std::uint64_t now) noexcept
{
    if (!hold_timer_active_) {
        return;
    }
    if (now < hold_deadline_us_) {
        return;
    }
    coil_.release();
    planner_.forceReleased(ReleaseReason::HoldExpired);
    hold_timer_active_ = false;
}

void AxisExecutor::publish() noexcept
{
    status_ = planner_.status();
    status_.energized = coil_.isEnergized();
}

AxisStatus AxisExecutor::status() const noexcept
{
    AxisStatus copy = status_;
    copy.energized = coil_.isEnergized();
    return copy;
}

std::uint32_t AxisExecutor::nextWaitUs() const noexcept
{
    const std::uint64_t now = alarm_.nowUs();
    const std::uint64_t kInfinite = std::numeric_limits<std::uint32_t>::max();

    std::uint64_t wait = kInfinite;
    if (planner_.hasPending()) {
        const std::uint64_t target = pending_deadline_us_ + params_.watchdog_us;
        wait = target > now ? target - now : 0U;
    } else if (hold_timer_active_) {
        wait = hold_deadline_us_ > now ? hold_deadline_us_ - now : 0U;
    } else {
        return static_cast<std::uint32_t>(kInfinite);
    }

    if (wait > kInfinite) {
        wait = kInfinite;
    }
    return static_cast<std::uint32_t>(wait);
}

}  // namespace motion
