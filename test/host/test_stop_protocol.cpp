/**
 * @file test_stop_protocol.cpp
 * @brief Подтверждение останова по тикету: старый покой не проходит.
 */

#include "check.hpp"
#include "control/stop_protocol.hpp"

namespace {

using control::CommandTicket;
using control::Delivery;
using control::StopMode;
using control::StopObservation;
using control::StopProtocol;
using control::StopReason;
using motion::AxisId;
using motion::AxisState;
using state::Micros;

constexpr std::uint32_t kEpoch = 7;

Delivery deliveryWith(std::uint64_t id)
{
    Delivery delivery{};
    delivery.accepted = true;
    delivery.notified = true;
    delivery.ticket.id = id;
    delivery.ticket.epoch = kEpoch;
    return delivery;
}

motion::AxisStatus restStatus(std::uint64_t applied_id)
{
    motion::AxisStatus status{};
    status.state = AxisState::Released;
    status.rate_steps_s = 0.0F;
    status.target_rate_steps_s = 0.0F;
    status.energized = false;
    status.command_epoch = kEpoch;
    status.last_applied_command_id = applied_id;
    return status;
}

StopProtocol makeProtocol()
{
    StopProtocol protocol;
    protocol.configure({1000});
    return protocol;
}

void beginSmooth(StopProtocol &protocol, Micros now)
{
    StopProtocol::Start start{};
    start.reason = StopReason::InputLoss;
    start.mode = StopMode::Smooth;
    start.now_us = now;
    start.baseline_epoch[0] = kEpoch;
    start.baseline_epoch[1] = kEpoch;
    protocol.begin(start);
    const StopProtocol::Action action = protocol.next();
    CHECK(action.send[0]);
    CHECK(action.send[1]);
    protocol.noteSent(AxisId::Azimuth, deliveryWith(10));
    protocol.noteSent(AxisId::Altitude, deliveryWith(11));
}

void staleRestIsNotConfirmation()
{
    StopProtocol protocol = makeProtocol();
    beginSmooth(protocol, 100);

    StopObservation obs[2]{};
    obs[0].status = restStatus(10);
    obs[1].status = restStatus(11);
    // Снимок прочитан до запроса останова.
    obs[0].polled_us = 90;
    obs[1].polled_us = 90;
    CHECK(protocol.check(obs, 100) == StopProtocol::Decision::Pending);

    obs[0].polled_us = 150;
    obs[1].polled_us = 150;
    CHECK(protocol.check(obs, 150) == StopProtocol::Decision::Confirmed);
}

void mismatchedTicketIsNotConfirmation()
{
    StopProtocol protocol = makeProtocol();
    beginSmooth(protocol, 100);

    StopObservation obs[2]{};
    obs[0].status = restStatus(9);   // чужой ack
    obs[1].status = restStatus(11);
    obs[0].polled_us = 200;
    obs[1].polled_us = 200;
    CHECK(protocol.check(obs, 200) == StopProtocol::Decision::Pending);
}

void releasedAxisNeedsTicket()
{
    StopProtocol protocol = makeProtocol();
    beginSmooth(protocol, 100);

    // Released с обесточенными фазами может снова принять команду,
    // поэтому покой надо доказывать тикетом.
    motion::AxisStatus released = restStatus(0);
    CHECK(!StopProtocol::axisCannotMove(released));

    StopObservation obs[2]{};
    obs[0].status = released;
    obs[1].status = released;
    obs[0].polled_us = 200;
    obs[1].polled_us = 200;
    CHECK(protocol.check(obs, 200) == StopProtocol::Decision::Pending);
}

void disabledAxisConfirmsWithoutTicket()
{
    StopProtocol protocol = makeProtocol();
    beginSmooth(protocol, 100);

    motion::AxisStatus disabled{};
    disabled.state = AxisState::Disabled;
    disabled.energized = false;
    disabled.rate_steps_s = 0.0F;
    CHECK(StopProtocol::axisCannotMove(disabled));

    StopObservation obs[2]{};
    obs[0].status = disabled;
    obs[1].status = disabled;
    obs[0].polled_us = 200;
    obs[1].polled_us = 200;
    CHECK(protocol.check(obs, 200) == StopProtocol::Decision::Confirmed);
}

void escalationSequence()
{
    StopProtocol protocol = makeProtocol();
    beginSmooth(protocol, 100);

    StopObservation obs[2]{};
    obs[0].status = restStatus(0);
    obs[1].status = restStatus(0);
    obs[0].polled_us = 5000;
    obs[1].polled_us = 5000;
    CHECK(protocol.check(obs, 1200) == StopProtocol::Decision::Escalate);
    CHECK(protocol.escalate(1200));
    CHECK(protocol.mode() == StopMode::Immediate);

    const StopProtocol::Action action = protocol.next();
    CHECK(action.mode == StopMode::Immediate);
    CHECK(action.send[0]);
    protocol.noteSent(AxisId::Azimuth, deliveryWith(20));
    protocol.noteSent(AxisId::Altitude, deliveryWith(21));

    CHECK(protocol.check(obs, 1300) == StopProtocol::Decision::Pending);
    CHECK(protocol.check(obs, 2400) == StopProtocol::Decision::Failed);
}

void immediateConfirmsByTicket()
{
    StopProtocol protocol = makeProtocol();
    StopProtocol::Start start{};
    start.reason = StopReason::MotorFault;
    start.mode = StopMode::Immediate;
    start.now_us = 100;
    start.baseline_epoch[0] = kEpoch;
    start.baseline_epoch[1] = kEpoch;
    protocol.begin(start);

    const StopProtocol::Action action = protocol.next();
    CHECK(action.mode == StopMode::Immediate);
    protocol.noteSent(AxisId::Azimuth, deliveryWith(30));
    protocol.noteSent(AxisId::Altitude, deliveryWith(31));

    StopObservation obs[2]{};
    obs[0].status = restStatus(30);
    obs[1].status = restStatus(31);
    obs[0].polled_us = 200;
    obs[1].polled_us = 200;
    CHECK(protocol.check(obs, 200) == StopProtocol::Decision::Confirmed);
}

void failedDeliveryRetries()
{
    StopProtocol protocol = makeProtocol();
    beginSmooth(protocol, 100);

    Delivery rejected{};
    protocol.noteSent(AxisId::Azimuth, rejected);
    protocol.noteSent(AxisId::Altitude, deliveryWith(11));
    const StopProtocol::Action retry = protocol.next();
    CHECK(retry.send[0]);
    CHECK(!retry.send[1]);
}

}  // namespace

int main()
{
    staleRestIsNotConfirmation();
    mismatchedTicketIsNotConfirmation();
    releasedAxisNeedsTicket();
    disabledAxisConfirmsWithoutTicket();
    escalationSequence();
    immediateConfirmsByTicket();
    failedDeliveryRetries();
    return TEST_RESULT();
}
