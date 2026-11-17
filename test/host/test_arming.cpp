/**
 * @file test_arming.cpp
 * @brief Автомат фаз и защёлок: разрешение, останов, восстановление.
 */

#include "check.hpp"
#include "control/arming.hpp"

namespace {

using control::ArmingDecision;
using control::ArmingInputs;
using control::ArmingMachine;
using state::ControlPhase;
using state::LatchCode;

ArmingMachine makeMachine()
{
    ArmingMachine machine;
    machine.configure({3});
    return machine;
}

void activate(ArmingMachine &machine)
{
    ArmingInputs inputs{};
    inputs.permit = true;
    (void)machine.step(inputs);  // Disarmed -> WaitingForNeutral
    inputs.neutral_confirmed = true;
    const ArmingDecision enabling = machine.step(inputs);
    CHECK(enabling.send_enable);
    CHECK(machine.phase() == ControlPhase::Enabling);
    inputs.axes_ready = true;
    (void)machine.step(inputs);
    CHECK(machine.phase() == ControlPhase::Active);
}

void disarmedUntilPermit()
{
    ArmingMachine machine = makeMachine();
    const ArmingDecision decision = machine.step(ArmingInputs{});
    CHECK(decision.phase == ControlPhase::Disarmed);
    CHECK(!decision.allow_nonzero);
}

void enableThenActive()
{
    ArmingMachine machine = makeMachine();
    activate(machine);
    CHECK(machine.latch() == LatchCode::None);
}

void allowNonzeroOnlyInActive()
{
    ArmingMachine machine = makeMachine();
    ArmingInputs inputs{};
    inputs.permit = true;
    inputs.neutral_confirmed = true;
    inputs.axes_ready = true;
    inputs.input_usable = true;

    const ArmingDecision waiting = machine.step(inputs);
    CHECK(waiting.phase == ControlPhase::WaitingForNeutral);
    CHECK(!waiting.allow_nonzero);

    const ArmingDecision enabling = machine.step(inputs);
    CHECK(enabling.phase == ControlPhase::Enabling);
    CHECK(enabling.send_enable);
    CHECK(!enabling.allow_nonzero);

    const ArmingDecision active = machine.step(inputs);
    CHECK(active.phase == ControlPhase::Active);
    CHECK(active.allow_nonzero);
}

void inputLossStartsStop()
{
    ArmingMachine machine = makeMachine();
    activate(machine);

    ArmingInputs inputs{};
    inputs.permit = true;
    inputs.input_usable = false;
    const ArmingDecision decision = machine.step(inputs);
    CHECK(decision.phase == ControlPhase::Stopping);
    CHECK(decision.request_stop);
    CHECK(decision.revoke_ownership);
    CHECK(decision.stop_reason == control::StopReason::InputLoss);
    CHECK(!decision.allow_nonzero);

    inputs.input_usable = true;
    inputs.stopped_confirmed = true;
    const ArmingDecision back = machine.step(inputs);
    CHECK(back.phase == ControlPhase::WaitingForNeutral);
}

void disarmDuringMotion()
{
    ArmingMachine machine = makeMachine();
    activate(machine);

    ArmingInputs inputs{};
    inputs.permit = false;
    inputs.input_usable = true;
    const ArmingDecision decision = machine.step(inputs);
    CHECK(decision.phase == ControlPhase::Stopping);
    CHECK(decision.stop_reason == control::StopReason::Disarm);
    CHECK(decision.request_stop);
}

void motorFaultLatches()
{
    ArmingMachine machine = makeMachine();
    activate(machine);

    ArmingInputs inputs{};
    inputs.permit = true;
    inputs.input_usable = true;
    inputs.motor_fault = true;
    const ArmingDecision decision = machine.step(inputs);
    CHECK(decision.latch == LatchCode::MotorFault);
    CHECK(decision.phase == ControlPhase::Stopping);

    inputs.motor_fault = false;
    inputs.stop_failed = true;
    const ArmingDecision failed = machine.step(inputs);
    CHECK(failed.phase == ControlPhase::Disarmed);
    CHECK(failed.latch == LatchCode::MotorFault);  // защёлка не подменяется
}

void stopUnconfirmedLatches()
{
    ArmingMachine machine = makeMachine();
    activate(machine);

    ArmingInputs inputs{};
    inputs.permit = true;
    inputs.input_usable = false;
    (void)machine.step(inputs);  // Stopping

    inputs.stop_failed = true;
    const ArmingDecision decision = machine.step(inputs);
    CHECK(decision.phase == ControlPhase::Disarmed);
    CHECK(decision.latch == LatchCode::StopUnconfirmed);
}

void latchSurvivesReset()
{
    ArmingMachine machine = makeMachine();
    machine.restoreLatch(LatchCode::StopUnconfirmed);

    ArmingInputs inputs{};
    inputs.permit = true;
    inputs.neutral_confirmed = true;
    inputs.axes_ready = true;
    ArmingDecision decision = machine.step(inputs);
    CHECK(decision.phase == ControlPhase::Stopping);  // движения нет
    CHECK(!decision.allow_nonzero);
    CHECK(decision.request_stop);

    inputs.stopped_confirmed = true;
    decision = machine.step(inputs);
    CHECK(decision.phase == ControlPhase::Disarmed);
    CHECK(decision.latch == LatchCode::StopUnconfirmed);

    machine.clearLatch();
    decision = machine.step(inputs);
    CHECK(decision.phase == ControlPhase::WaitingForNeutral);
}

void deliveryFailureLatches()
{
    ArmingMachine machine = makeMachine();
    activate(machine);

    ArmingInputs inputs{};
    inputs.permit = true;
    inputs.input_usable = true;
    inputs.rate_delivery_failed = true;
    const ArmingDecision decision = machine.step(inputs);
    CHECK(decision.latch == LatchCode::DeliveryFailed);
    CHECK(decision.phase == ControlPhase::Stopping);
    CHECK(decision.stop_reason == control::StopReason::DeliveryLimit);
}

void handoverPath()
{
    ArmingMachine machine = makeMachine();
    activate(machine);

    ArmingInputs inputs{};
    inputs.permit = true;
    inputs.input_usable = true;
    inputs.handover_pending = true;
    const ArmingDecision start = machine.step(inputs);
    CHECK(start.phase == ControlPhase::Stopping);
    CHECK(start.stop_reason == control::StopReason::Handover);

    inputs.stopped_confirmed = true;
    inputs.handover_valid = true;
    const ArmingDecision done = machine.step(inputs);
    CHECK(done.phase == ControlPhase::Active);
    CHECK(done.handover_complete);
}

void handoverRejectedReturnsToNeutral()
{
    ArmingMachine machine = makeMachine();
    activate(machine);

    ArmingInputs inputs{};
    inputs.permit = true;
    inputs.input_usable = true;
    inputs.handover_pending = true;
    (void)machine.step(inputs);

    inputs.stopped_confirmed = true;
    inputs.handover_valid = false;
    const ArmingDecision decision = machine.step(inputs);
    CHECK(decision.phase == ControlPhase::WaitingForNeutral);
    CHECK(!decision.handover_complete);
}

void shutdownFromAnyPhase()
{
    // Disarmed
    {
        ArmingMachine machine = makeMachine();
        ArmingInputs inputs{};
        inputs.stop_requested = true;
        const ArmingDecision decision = machine.step(inputs);
        CHECK(decision.phase == ControlPhase::Stopping);
        CHECK(decision.stop_reason == control::StopReason::SafetyStop);
    }
    // WaitingForNeutral
    {
        ArmingMachine machine = makeMachine();
        ArmingInputs inputs{};
        inputs.permit = true;
        (void)machine.step(inputs);
        inputs.stop_requested = true;
        const ArmingDecision decision = machine.step(inputs);
        CHECK(decision.phase == ControlPhase::Stopping);
    }
    // Enabling
    {
        ArmingMachine machine = makeMachine();
        ArmingInputs inputs{};
        inputs.permit = true;
        (void)machine.step(inputs);
        inputs.neutral_confirmed = true;
        (void)machine.step(inputs);
        CHECK(machine.phase() == ControlPhase::Enabling);
        inputs.stop_requested = true;
        const ArmingDecision decision = machine.step(inputs);
        CHECK(decision.phase == ControlPhase::Stopping);
    }
    // Active и Stopping уже покрыты выше.
}

void calibrationSourceBranches()
{
    ArmingMachine machine = makeMachine();
    ArmingInputs inputs{};
    inputs.permit = true;
    inputs.selected_source = state::SourceId::Calibration;
    // Джойстик исправен, но выбран источник калибровки.
    inputs.input_usable = true;
    inputs.neutral_confirmed = true;
    (void)machine.step(inputs);
    CHECK(machine.phase() == ControlPhase::WaitingForNeutral);

    // Пригодность калибровки не подтверждена: движения нет.
    inputs.permit = true;
    const ArmingDecision blocked = machine.step(inputs);
    CHECK(blocked.phase == ControlPhase::WaitingForNeutral);
    CHECK(!blocked.allow_nonzero);

    inputs.calibration_input_ready = true;
    const ArmingDecision enabling = machine.step(inputs);
    CHECK(enabling.send_enable);
    inputs.axes_ready = true;
    const ArmingDecision active = machine.step(inputs);
    CHECK(active.phase == ControlPhase::Active);
    CHECK(active.allow_nonzero);

    // Истёкший lease калибровки даёт InputLoss, даже если джойстик в норме.
    inputs.calibration_input_ready = false;
    const ArmingDecision lost = machine.step(inputs);
    CHECK(lost.phase == ControlPhase::Stopping);
    CHECK(lost.stop_reason == control::StopReason::InputLoss);
    CHECK(lost.request_stop);
}

void joystickHealthyDoesNotMaskCalibrationFailure()
{
    ArmingMachine machine = makeMachine();
    ArmingInputs inputs{};
    inputs.permit = true;
    inputs.selected_source = state::SourceId::Joystick;
    inputs.input_usable = true;
    inputs.neutral_confirmed = true;
    (void)machine.step(inputs);
    (void)machine.step(inputs);
    inputs.axes_ready = true;
    const ArmingDecision active = machine.step(inputs);
    CHECK(active.phase == ControlPhase::Active);

    // Калибровка не выбрана, поэтому её готовность не влияет.
    inputs.calibration_input_ready = false;
    const ArmingDecision still = machine.step(inputs);
    CHECK(still.phase == ControlPhase::Active);
    CHECK(still.allow_nonzero);
}

void activeWithoutUsableInputHasNoNonzero()
{
    ArmingMachine machine = makeMachine();
    activate(machine);
    ArmingInputs inputs{};
    inputs.permit = true;
    inputs.input_usable = false;
    (void)machine.step(inputs);
    // После входа в Stopping ненулевых уставок нет.
    CHECK(!machine.step(inputs).allow_nonzero);
}

}  // namespace

int main()
{
    disarmedUntilPermit();
    enableThenActive();
    allowNonzeroOnlyInActive();
    inputLossStartsStop();
    disarmDuringMotion();
    motorFaultLatches();
    stopUnconfirmedLatches();
    latchSurvivesReset();
    deliveryFailureLatches();
    handoverPath();
    handoverRejectedReturnsToNeutral();
    shutdownFromAnyPhase();
    calibrationSourceBranches();
    joystickHealthyDoesNotMaskCalibrationFailure();
    activeWithoutUsableInputHasNoNonzero();
    return TEST_RESULT();
}
