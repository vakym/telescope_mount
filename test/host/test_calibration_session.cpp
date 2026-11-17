#include "calibration/session.hpp"
#include "check.hpp"

using namespace calibration;

namespace {

CalibrationParams params()
{
    CalibrationParams p;
    p.min_revolutions = 1;
    p.max_revolutions = 20;
    p.min_measurements = 2;
    p.max_measurements = 5;
    p.max_spread_ppm = 5000;
    return p;
}

SessionConfig config()
{
    SessionConfig c;
    c.axis = motion::AxisId::Azimuth;
    c.direction = motion::Direction::Forward;
    c.revolutions = 1;
    c.repeats = 2;
    return c;
}

ReferenceAttestation attestation(std::int64_t position, std::uint64_t ticket)
{
    ReferenceAttestation a;
    a.valid = true;
    a.position = position;
    a.ticket = ticket;
    a.epoch = 1;
    a.position_trusted = true;
    a.energized = true;
    a.fault = false;
    return a;
}

void passStart(Session &s, std::int64_t position, std::uint64_t ticket)
{
    CHECK(s.requestStop());
    CHECK(s.confirmStop(attestation(position, ticket), 1, true));
    CHECK(s.state() == SessionState::StartReady);
    CHECK(s.confirmStart());
    CHECK(s.state() == SessionState::Measuring);
}

}  // namespace

int main()
{
    Session s;
    CHECK(s.state() == SessionState::Inactive);
    CHECK(s.start(config(), params()));
    CHECK(s.state() == SessionState::Preparing);
    CHECK(s.ownerReady());
    CHECK(s.state() == SessionState::InitialPositioning);

    const std::uint32_t gen_before_stop = s.motionGeneration();
    CHECK(s.requestStop());
    CHECK(s.state() == SessionState::AwaitingStartStop);
    CHECK(s.motionGeneration() == gen_before_stop + 1);
    CHECK(!s.requestStop());  // повтор не создаёт новую операцию
    CHECK(s.motionGeneration() == gen_before_stop + 1);

    CHECK(s.confirmStop(attestation(1000, 1), 1, true));
    CHECK(s.state() == SessionState::StartReady);
    CHECK(s.confirmStart());
    CHECK(s.state() == SessionState::Measuring);

    CHECK(s.requestStop());
    CHECK(s.confirmStop(attestation(5096, 2), 1, true));
    CHECK(s.state() == SessionState::EndReady);
    CHECK(s.confirmEnd());
    CHECK(s.state() == SessionState::StartReady);
    CHECK(s.measurementCount() == 1);

    CHECK(s.confirmStart());
    CHECK(s.requestStop());
    CHECK(s.confirmStop(attestation(9192, 3), 1, true));
    CHECK(s.confirmEnd());
    CHECK(s.state() == SessionState::Review);
    CHECK(s.measurementCount() == 2);
    CHECK(s.result().ok);
    CHECK(s.result().sample_count == 2);
    CHECK(s.result().total_revolutions == 2);
    CHECK_NEAR(s.result().degrees_per_half_step, 360.0 / 4096.0, 1e-12);

    CHECK(s.apply(false) == ApplyResult::Ok);
    CHECK(s.state() == SessionState::Applying);
    s.beginSaving();
    CHECK(s.state() == SessionState::Saving);
    s.saveVerified(s.result());
    CHECK(s.state() == SessionState::Saved);
    CHECK(s.result().ok);
    s.close();
    CHECK(s.state() == SessionState::Inactive);

    // Нулевая разность аннулирует измерение.
    Session zero;
    CHECK(zero.start(config(), params()));
    CHECK(zero.ownerReady());
    passStart(zero, 100, 10);
    CHECK(zero.requestStop());
    CHECK(zero.confirmStop(attestation(100, 11), 1, true));
    CHECK(!zero.confirmEnd());
    CHECK(zero.state() == SessionState::InitialPositioning);
    CHECK(zero.markRecheckRequired());
    CHECK(zero.lastInvalidReason() == InvalidReason::ZeroDelta);

    // Потеря удержания в StartReady требует повторной проверки метки.
    Session hold;
    CHECK(hold.start(config(), params()));
    CHECK(hold.ownerReady());
    CHECK(hold.requestStop());
    CHECK(hold.confirmStop(attestation(1000, 1), 1, true));
    hold.holdLost();
    CHECK(hold.state() == SessionState::InitialPositioning);
    CHECK(hold.markRecheckRequired());
    CHECK(!hold.holdActive());

    // Истечение lease переводит в Error и увеличивает поколение.
    Session expired;
    CHECK(expired.start(config(), params()));
    CHECK(expired.ownerReady());
    const std::uint32_t expired_generation = expired.motionGeneration();
    expired.leaseExpired();
    CHECK(expired.state() == SessionState::Error);
    CHECK(expired.error() == SessionError::ClientLeaseExpired);
    CHECK(expired.motionGeneration() > expired_generation);

    // Реверс во время измерения — ошибка.
    Session reverse;
    CHECK(reverse.start(config(), params()));
    CHECK(reverse.ownerReady());
    passStart(reverse, 1000, 1);
    CHECK(!reverse.noteMove(motion::Direction::Backward, reverse.motionGeneration()));
    CHECK(reverse.state() == SessionState::Error);
    CHECK(reverse.error() == SessionError::DirectionViolation);

    // Доводка из EndReady аннулирует конечную точку и возвращает в Measuring.
    Session adjust;
    CHECK(adjust.start(config(), params()));
    CHECK(adjust.ownerReady());
    passStart(adjust, 1000, 1);
    CHECK(adjust.requestStop());
    CHECK(adjust.confirmStop(attestation(5096, 2), 1, true));
    CHECK(adjust.state() == SessionState::EndReady);
    CHECK(adjust.noteMove(motion::Direction::Forward, adjust.motionGeneration()));
    CHECK(adjust.state() == SessionState::Measuring);
    CHECK(!adjust.confirmEnd());
    // Новая конечная точка появляется только после нового останова.
    CHECK(adjust.requestStop());
    CHECK(adjust.confirmStop(attestation(5200, 3), 1, true));
    CHECK(adjust.confirmEnd());
    CHECK(adjust.measurementCount() == 1);

    // Повтор сохранения после ошибки использует тот же результат.
    Session retry;
    CHECK(retry.start(config(), params()));
    CHECK(retry.ownerReady());
    passStart(retry, 0, 1);
    CHECK(retry.requestStop());
    CHECK(retry.confirmStop(attestation(4096, 2), 1, true));
    CHECK(retry.confirmEnd());
    CHECK(retry.state() == SessionState::StartReady);
    CHECK(retry.confirmStart());
    CHECK(retry.requestStop());
    CHECK(retry.confirmStop(attestation(8192, 3), 1, true));
    CHECK(retry.confirmEnd());
    CHECK(retry.state() == SessionState::Review);
    CHECK(retry.apply(false) == ApplyResult::Ok);
    retry.beginSaving();
    retry.saveFailed();
    CHECK(retry.state() == SessionState::SaveFailed);
    CHECK(retry.apply(false) == ApplyResult::Ok);
    CHECK(retry.state() == SessionState::Applying);

    // Удаление перенумеровывает записи: показанный номер == индекс.
    Session renumber;
    CHECK(renumber.start(config(), params()));
    CHECK(renumber.ownerReady());
    passStart(renumber, 0, 1);
    CHECK(renumber.requestStop());
    CHECK(renumber.confirmStop(attestation(4096, 2), 1, true));
    CHECK(renumber.confirmEnd());
    CHECK(renumber.confirmStart());
    CHECK(renumber.requestStop());
    CHECK(renumber.confirmStop(attestation(8192, 3), 1, true));
    CHECK(renumber.confirmEnd());
    CHECK(renumber.state() == SessionState::Review);
    CHECK(renumber.measurementCount() == 2);
    CHECK(renumber.deleteMeasurement(0));
    CHECK(renumber.measurementCount() == 1);
    CHECK(renumber.measurements()[0].index == 0U);

    // Эпоха исполнителя не должна меняться между опорными точками.
    Session epoch;
    CHECK(epoch.start(config(), params()));
    CHECK(epoch.ownerReady());
    passStart(epoch, 0, 1);
    CHECK(epoch.requestStop());
    ReferenceAttestation moved_epoch = attestation(4096, 2);
    moved_epoch.epoch = 2;
    CHECK(!epoch.confirmStop(moved_epoch, 2, true));
    CHECK(epoch.state() == SessionState::InitialPositioning);
    CHECK(epoch.lastInvalidReason() == InvalidReason::EpochChanged);

    // Stop не подтверждается без удержания.
    Session no_hold;
    CHECK(no_hold.start(config(), params()));
    CHECK(no_hold.ownerReady());
    CHECK(no_hold.requestStop());
    ReferenceAttestation no_power = attestation(1000, 1);
    no_power.energized = false;
    CHECK(!no_hold.confirmStop(no_power, 1, true));
    CHECK(no_hold.state() == SessionState::InitialPositioning);
    CHECK(no_hold.markRecheckRequired());

    return TEST_RESULT();
}
