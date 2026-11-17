/**
 * @file arming.hpp
 * @brief Автомат разрешения движения: фаза, защёлка и permit раздельно.
 *
 * Инварианты:
 *  I1  в фазе Stopping доставка останова и наблюдение разрешены всегда;
 *  I2  ненулевые уставки — только permit && latch == None && phase == Active;
 *  I3  вход в Active — только через Enabling с подтверждённой готовностью;
 *  I4  защёлка снимается только явно, не автоматом и не повторным start();
 *  I5  ни один переход не отменяет активный протокол останова;
 *  I6  disarm() и stop() не означают «ничего не отправляем».
 */

#pragma once

#include <cstdint>

#include "control/stop_protocol.hpp"
#include "state/channels.hpp"

namespace control {

struct ArmingConfig {
    /// Предел подряд идущих отказов доставки перед остановом.
    std::uint32_t delivery_failure_limit = 3;
};

struct ArmingInputs {
    bool permit = false;             ///< разрешение выдано
    bool neutral_confirmed = false;  ///< свежая подтверждённая нейтраль
    bool input_usable = false;       ///< уставки формируются (ok или grace)
    /// Текущий владелец арбитража. Проверка входа выполняется по выбранному
    /// источнику, а не объединением: исправный джойстик не маскирует отказ
    /// калибровки и наоборот.
    state::SourceId selected_source = state::SourceId::None;
    /// Пригодность калибровочного входа (владение + сессия + удержание + lease).
    bool calibration_input_ready = false;
    bool motor_fault = false;        ///< Fault хотя бы одной оси
    bool axes_ready = false;         ///< свежий опрос: нет Fault и не Disabled
    /// Ось перестала быть готовой уже после разрешения (Disabled/Fault).
    bool axis_lost = false;
    bool enable_failed = false;      ///< доставка Enable исчерпала попытки
    bool rate_delivery_failed = false;
    bool stop_requested = false;  ///< safety stop или shutdown
    /// Штатный останов калибровки: не отзывает владение и не снимает permit.
    bool calibration_stop = false;
    bool stopped_confirmed = false;
    bool stop_escalate = false;
    bool stop_failed = false;
    bool handover_pending = false;
    bool handover_valid = false;
};

struct ArmingDecision {
    state::ControlPhase phase = state::ControlPhase::Disarmed;
    state::LatchCode latch = state::LatchCode::None;
    bool send_enable = false;
    bool request_stop = false;
    StopReason stop_reason = StopReason::InputLoss;
    bool allow_nonzero = false;
    bool revoke_ownership = false;
    bool handover_complete = false;
};

class ArmingMachine {
public:
    void configure(const ArmingConfig &config) noexcept;
    void reset() noexcept;

    /// Снятие защёлки после успешного протокола восстановления.
    void clearLatch() noexcept;

    /// Восстановление сохранённой защёлки после перезапуска службы:
    /// latch переживает start()/stop() и не снимается сбросом объектов.
    void restoreLatch(state::LatchCode latch) noexcept;

    /// Протокол останова начат по указанной причине.
    void noteStopIssued(StopReason reason) noexcept;

    ArmingDecision step(const ArmingInputs &in) noexcept;

    state::ControlPhase phase() const noexcept { return phase_; }
    state::LatchCode latch() const noexcept { return latch_; }
    StopReason stopReason() const noexcept { return reason_; }

private:
    void beginStopping(StopReason reason) noexcept;

    ArmingConfig config_{};
    state::ControlPhase phase_ = state::ControlPhase::Disarmed;
    state::LatchCode latch_ = state::LatchCode::None;
    StopReason reason_ = StopReason::InputLoss;
};

}  // namespace control
