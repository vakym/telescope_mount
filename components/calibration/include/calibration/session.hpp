/**
 * @file session.hpp
 * @brief Чистый автомат одной сессии калибровки оси.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "calibration/math.hpp"
#include "calibration/types.hpp"

namespace calibration {

struct ReferenceAttestation {
    bool valid = false;
    std::int64_t position = 0;
    std::uint64_t ticket = 0;
    std::uint32_t epoch = 0;
    bool position_trusted = false;
    bool energized = false;
    bool fault = false;
};

enum class ApplyResult : std::uint8_t {
    Ok,
    NotReady,
    RepeatabilityRejected,
};

class Session {
public:
    void reset() noexcept;

    bool start(const SessionConfig &config, const CalibrationParams &params) noexcept;
    bool ownerReady() noexcept;
    bool noteMove(motion::Direction direction, std::uint32_t generation) noexcept;
    bool requestStop() noexcept;
    bool confirmStop(const ReferenceAttestation &attestation,
                     std::uint32_t current_epoch, bool reference_ready) noexcept;
    bool confirmStart() noexcept;
    bool confirmEnd() noexcept;
    bool resume() noexcept;
    bool abandon() noexcept;
    bool deleteMeasurement(std::size_t index) noexcept;
    bool addRepeat() noexcept;
    ApplyResult apply(bool acknowledge_low_repeatability) noexcept;
    void beginSaving() noexcept;
    void saveVerified(const CombinedResult &result) noexcept;
    void saveFailed() noexcept;
    void saveUncertain() noexcept;
    bool cancel() noexcept;
    void cancelStopConfirmed() noexcept;
    void close() noexcept;
    void fail(SessionError error) noexcept;
    void leaseExpired() noexcept;
    void holdLost() noexcept;

    SessionState state() const noexcept { return state_; }
    SessionError error() const noexcept { return error_; }
    bool active() const noexcept;
    bool terminal() const noexcept;
    std::uint32_t revision() const noexcept { return revision_; }
    std::uint32_t motionGeneration() const noexcept { return motion_generation_; }
    const SessionConfig &config() const noexcept { return config_; }
    const CalibrationParams &params() const noexcept { return params_; }
    const Measurement *measurements() const noexcept { return measurements_.data(); }
    std::size_t measurementCount() const noexcept { return measurement_count_; }
    const CombinedResult &result() const noexcept { return result_; }
    bool markRecheckRequired() const noexcept { return mark_recheck_required_; }
    bool holdActive() const noexcept { return hold_active_; }
    std::uint32_t currentIndex() const noexcept { return measurement_count_; }
    std::uint32_t currentRevolutions() const noexcept { return config_.revolutions; }
    std::int64_t currentStart() const noexcept { return current_start_.position; }
    std::int64_t currentEnd() const noexcept { return current_end_.position; }
    std::uint64_t currentStartTicket() const noexcept { return current_start_.ticket; }
    std::uint64_t currentStopTicket() const noexcept { return current_end_.ticket; }
    InvalidReason lastInvalidReason() const noexcept { return last_invalid_; }
    std::uint32_t lastSaveSequence() const noexcept { return last_save_sequence_; }

private:
    bool movingState() const noexcept;
    void bumpRevision() noexcept;
    void revokePermission() noexcept;
    void recompute() noexcept;
    bool canConfirmReference(const ReferenceAttestation &attestation,
                             std::uint32_t current_epoch,
                             bool reference_ready) const noexcept;

    SessionState state_ = SessionState::Inactive;
    SessionError error_ = SessionError::None;
    SessionConfig config_{};
    CalibrationParams params_{};
    std::array<Measurement, kMaxMeasurements> measurements_{};
    std::size_t measurement_count_ = 0;
    CombinedResult result_{};
    ReferenceAttestation current_start_{};
    ReferenceAttestation current_end_{};
    InvalidReason last_invalid_ = InvalidReason::Ok;
    bool mark_recheck_required_ = false;
    bool hold_active_ = false;
    bool acknowledged_low_repeatability_ = false;
    std::uint32_t revision_ = 0;
    std::uint32_t motion_generation_ = 0;
    std::uint32_t last_save_sequence_ = 0;
};

}  // namespace calibration
