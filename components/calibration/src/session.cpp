#include "calibration/session.hpp"

#include <algorithm>
#include <cmath>

namespace calibration {

void Session::reset() noexcept
{
    state_ = SessionState::Inactive;
    error_ = SessionError::None;
    config_ = SessionConfig{};
    params_ = CalibrationParams{};
    measurements_.fill(Measurement{});
    measurement_count_ = 0;
    result_ = CombinedResult{};
    current_start_ = ReferenceAttestation{};
    current_end_ = ReferenceAttestation{};
    last_invalid_ = InvalidReason::Ok;
    mark_recheck_required_ = false;
    hold_active_ = false;
    acknowledged_low_repeatability_ = false;
    revision_ = 0;
    motion_generation_ = 0;
    last_save_sequence_ = 0;
}

bool Session::start(const SessionConfig &config, const CalibrationParams &params) noexcept
{
    if (state_ != SessionState::Inactive || !validConfig(config, params)) {
        return false;
    }
    reset();
    config_ = config;
    params_ = params;
    state_ = SessionState::Preparing;
    hold_active_ = false;
    revision_ = 1;
    ++motion_generation_;
    return true;
}

bool Session::ownerReady() noexcept
{
    if (state_ != SessionState::Preparing) {
        return false;
    }
    state_ = SessionState::InitialPositioning;
    hold_active_ = true;
    bumpRevision();
    return true;
}

bool Session::noteMove(motion::Direction direction, std::uint32_t generation) noexcept
{
    if (generation != motion_generation_) {
        return false;
    }
    switch (state_) {
        case SessionState::InitialPositioning:
            return true;
        case SessionState::StartReady:
            current_start_ = ReferenceAttestation{};
            state_ = SessionState::InitialPositioning;
            mark_recheck_required_ = false;
            bumpRevision();
            return true;
        case SessionState::Measuring:
            if (direction != config_.direction) {
                fail(SessionError::DirectionViolation);
                last_invalid_ = InvalidReason::Reversal;
                return false;
            }
            return true;
        case SessionState::EndReady:
            // Доводка после конечного останова: старая конечная точка
            // аннулируется, подтверждение снова требует нового останова.
            if (direction != config_.direction) {
                fail(SessionError::DirectionViolation);
                last_invalid_ = InvalidReason::Reversal;
                return false;
            }
            current_end_ = ReferenceAttestation{};
            state_ = SessionState::Measuring;
            hold_active_ = true;
            mark_recheck_required_ = false;
            bumpRevision();
            return true;
        default:
            return false;
    }
}

bool Session::requestStop() noexcept
{
    if (state_ == SessionState::InitialPositioning) {
        state_ = SessionState::AwaitingStartStop;
        revokePermission();
        bumpRevision();
        return true;
    }
    if (state_ == SessionState::Measuring) {
        state_ = SessionState::AwaitingEndStop;
        revokePermission();
        bumpRevision();
        return true;
    }
    return false;
}

bool Session::canConfirmReference(const ReferenceAttestation &attestation,
                                  std::uint32_t current_epoch,
                                  bool reference_ready) const noexcept
{
    if (!attestation.valid || !attestation.position_trusted ||
        attestation.fault || !attestation.energized) {
        return false;
    }
    if (current_epoch == 0U || attestation.epoch != current_epoch) {
        return false;
    }
    return reference_ready;
}

bool Session::confirmStop(const ReferenceAttestation &attestation,
                          std::uint32_t current_epoch, bool reference_ready) noexcept
{
    if (state_ != SessionState::AwaitingStartStop &&
        state_ != SessionState::AwaitingEndStop) {
        return false;
    }
    // Эпоха исполнителя не должна меняться внутри измерения: перезапуск оси
    // аннулирует пару опорных точек, даже если текущий снимок согласован.
    const bool epoch_broken =
        state_ == SessionState::AwaitingEndStop && current_start_.valid &&
        attestation.valid && attestation.epoch != current_start_.epoch;
    if (epoch_broken || !canConfirmReference(attestation, current_epoch, reference_ready)) {
        last_invalid_ = epoch_broken                ? InvalidReason::EpochChanged
                        : !attestation.position_trusted ? InvalidReason::PositionUntrusted
                        : attestation.fault            ? InvalidReason::Fault
                        : !attestation.energized       ? InvalidReason::HoldLost
                                                       : InvalidReason::StopUnconfirmed;
        state_ = SessionState::InitialPositioning;
        mark_recheck_required_ = true;
        hold_active_ = false;
        current_start_ = ReferenceAttestation{};
        current_end_ = ReferenceAttestation{};
        bumpRevision();
        return false;
    }
    if (state_ == SessionState::AwaitingStartStop) {
        current_start_ = attestation;
        state_ = SessionState::StartReady;
        mark_recheck_required_ = false;
        hold_active_ = true;
        bumpRevision();
        return true;
    }
    current_end_ = attestation;
    state_ = SessionState::EndReady;
    hold_active_ = true;
    bumpRevision();
    return true;
}

bool Session::confirmStart() noexcept
{
    if (state_ != SessionState::StartReady || !current_start_.valid) {
        return false;
    }
    state_ = SessionState::Measuring;
    hold_active_ = true;
    bumpRevision();
    return true;
}

bool Session::confirmEnd() noexcept
{
    if (state_ != SessionState::EndReady || !current_start_.valid || !current_end_.valid) {
        return false;
    }
    if (current_start_.epoch == 0U || current_start_.epoch != current_end_.epoch) {
        // Эпоха исполнителя изменилась между точками: измерение недействительно.
        last_invalid_ = InvalidReason::EpochChanged;
        mark_recheck_required_ = true;
        current_start_ = ReferenceAttestation{};
        current_end_ = ReferenceAttestation{};
        state_ = SessionState::InitialPositioning;
        hold_active_ = true;
        bumpRevision();
        return false;
    }
    Measurement measurement = makeMeasurement(
        static_cast<std::uint32_t>(measurement_count_), config_.revolutions,
        current_start_.position, current_end_.position, config_.direction,
        current_start_.ticket, current_end_.ticket, current_end_.epoch, params_);
    if (!measurement.valid) {
        last_invalid_ = measurement.reason;
        mark_recheck_required_ = true;
        current_start_ = ReferenceAttestation{};
        current_end_ = ReferenceAttestation{};
        state_ = SessionState::InitialPositioning;
        bumpRevision();
        return false;
    }
    if (measurement_count_ >= measurements_.size()) {
        last_invalid_ = InvalidReason::InvalidResult;
        return false;
    }
    measurements_[measurement_count_++] = measurement;
    recompute();
    last_invalid_ = InvalidReason::Ok;
    mark_recheck_required_ = false;
    if (measurement_count_ >= config_.repeats) {
        state_ = SessionState::Review;
        current_start_ = ReferenceAttestation{};
        current_end_ = ReferenceAttestation{};
        hold_active_ = false;
    } else {
        current_start_ = current_end_;
        current_end_ = ReferenceAttestation{};
        state_ = SessionState::StartReady;
        hold_active_ = true;
    }
    bumpRevision();
    return true;
}

bool Session::resume() noexcept
{
    if (state_ != SessionState::EndReady) {
        return false;
    }
    current_end_ = ReferenceAttestation{};
    state_ = SessionState::Measuring;
    hold_active_ = true;
    bumpRevision();
    return true;
}

bool Session::abandon() noexcept
{
    if (state_ != SessionState::Measuring && state_ != SessionState::EndReady) {
        return false;
    }
    current_start_ = ReferenceAttestation{};
    current_end_ = ReferenceAttestation{};
    last_invalid_ = InvalidReason::ClientAbort;
    mark_recheck_required_ = true;
    state_ = SessionState::InitialPositioning;
    hold_active_ = true;
    bumpRevision();
    return true;
}

bool Session::deleteMeasurement(std::size_t index) noexcept
{
    if (state_ != SessionState::Review || index >= measurement_count_) {
        return false;
    }
    for (std::size_t i = index + 1; i < measurement_count_; ++i) {
        measurements_[i - 1] = measurements_[i];
    }
    --measurement_count_;
    measurements_[measurement_count_] = Measurement{};
    recompute();
    bumpRevision();
    return true;
}

bool Session::addRepeat() noexcept
{
    if (state_ != SessionState::Review) {
        return false;
    }
    current_start_ = ReferenceAttestation{};
    current_end_ = ReferenceAttestation{};
    state_ = SessionState::Preparing;
    hold_active_ = false;
    bumpRevision();
    return true;
}

ApplyResult Session::apply(bool acknowledge_low_repeatability) noexcept
{
    // Повторное сохранение после ошибки разрешено: рассчитанный результат
    // остаётся в памяти, повторять измерения не требуется.
    if (state_ != SessionState::Review && state_ != SessionState::SaveFailed &&
        state_ != SessionState::SaveUncertain) {
        return ApplyResult::NotReady;
    }
    if (!result_.ok || result_.sample_count < params_.min_measurements) {
        return ApplyResult::NotReady;
    }
    if (result_.repeatability == Repeatability::Warn &&
        !acknowledge_low_repeatability) {
        return ApplyResult::RepeatabilityRejected;
    }
    acknowledged_low_repeatability_ = acknowledge_low_repeatability;
    state_ = SessionState::Applying;
    error_ = SessionError::None;
    hold_active_ = false;
    bumpRevision();
    return ApplyResult::Ok;
}

void Session::beginSaving() noexcept
{
    if (state_ == SessionState::Applying) {
        state_ = SessionState::Saving;
        bumpRevision();
    }
}

void Session::saveVerified(const CombinedResult &result) noexcept
{
    if (state_ != SessionState::Saving) {
        return;
    }
    result_ = result;
    state_ = SessionState::Saved;
    ++last_save_sequence_;
    bumpRevision();
}

void Session::saveFailed() noexcept
{
    if (state_ == SessionState::Saving) {
        state_ = SessionState::SaveFailed;
        error_ = SessionError::SaveFailed;
        bumpRevision();
    }
}

void Session::saveUncertain() noexcept
{
    if (state_ == SessionState::Saving) {
        state_ = SessionState::SaveUncertain;
        error_ = SessionError::SaveUncertain;
        bumpRevision();
    }
}

bool Session::cancel() noexcept
{
    if (state_ == SessionState::Inactive || state_ == SessionState::Cancelled ||
        state_ == SessionState::Saved || state_ == SessionState::SaveFailed ||
        state_ == SessionState::SaveUncertain || state_ == SessionState::Error) {
        return false;
    }
    if (movingState()) {
        state_ = SessionState::Cancelling;
        revokePermission();
        hold_active_ = false;
        bumpRevision();
        return true;
    }
    state_ = SessionState::Cancelled;
    revokePermission();
    hold_active_ = false;
    bumpRevision();
    return true;
}

void Session::cancelStopConfirmed() noexcept
{
    if (state_ == SessionState::Cancelling) {
        state_ = SessionState::Cancelled;
        hold_active_ = false;
        bumpRevision();
    }
}

void Session::close() noexcept
{
    if (!terminal()) {
        return;
    }
    state_ = SessionState::Inactive;
    hold_active_ = false;
    bumpRevision();
}

void Session::fail(SessionError error) noexcept
{
    if (state_ == SessionState::Inactive) {
        return;
    }
    error_ = error;
    state_ = SessionState::Error;
    hold_active_ = false;
    mark_recheck_required_ = true;
    revokePermission();
    bumpRevision();
}

void Session::leaseExpired() noexcept
{
    if (state_ == SessionState::Measuring || state_ == SessionState::InitialPositioning ||
        state_ == SessionState::StartReady || state_ == SessionState::EndReady ||
        state_ == SessionState::AwaitingStartStop || state_ == SessionState::AwaitingEndStop) {
        last_invalid_ = InvalidReason::LeaseExpired;
        fail(SessionError::ClientLeaseExpired);
    }
}

void Session::holdLost() noexcept
{
    if (state_ == SessionState::StartReady || state_ == SessionState::Measuring ||
        state_ == SessionState::EndReady || state_ == SessionState::InitialPositioning) {
        last_invalid_ = InvalidReason::HoldLost;
        current_start_ = ReferenceAttestation{};
        current_end_ = ReferenceAttestation{};
        state_ = SessionState::InitialPositioning;
        hold_active_ = false;
        mark_recheck_required_ = true;
        bumpRevision();
    }
}

bool Session::active() const noexcept
{
    return state_ != SessionState::Inactive && state_ != SessionState::Cancelled;
}

bool Session::terminal() const noexcept
{
    return state_ == SessionState::Saved || state_ == SessionState::SaveFailed ||
           state_ == SessionState::SaveUncertain || state_ == SessionState::Cancelled ||
           state_ == SessionState::Error;
}

bool Session::movingState() const noexcept
{
    return state_ == SessionState::InitialPositioning ||
           state_ == SessionState::StartReady ||
           state_ == SessionState::Measuring ||
           state_ == SessionState::AwaitingStartStop ||
           state_ == SessionState::AwaitingEndStop ||
           state_ == SessionState::EndReady;
}

void Session::bumpRevision() noexcept
{
    ++revision_;
}

void Session::revokePermission() noexcept
{
    ++motion_generation_;
}

void Session::recompute() noexcept
{
    result_ = combine(measurements_.data(), measurement_count_, params_);
}

}  // namespace calibration
