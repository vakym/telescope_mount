/**
 * @file control_port.hpp
 * @brief Адаптер calibration::CalibrationPort над ControlService.
 *
 * Прямого доступа к MotorService нет: все команды идут через управляющий
 * слой, который остаётся единственным исполнителем моторных операций.
 */

#pragma once

#include <cstdint>

#include "calibration/port.hpp"
#include "control_hal/control_service.hpp"
#include "state_hal/state_hub.hpp"

namespace calibration_hal {

class ControlPort final : public calibration::CalibrationPort {
public:
    ControlPort(control_hal::ControlService &control, state_hal::StateHub &hub) noexcept;

    bool claim(std::uint32_t session) override;
    void release(std::uint32_t session) override;
    void setInputReady(bool ready, std::uint32_t session,
                       calibration::Micros valid_until_us) override;
    bool beginHold(motion::AxisId axis, std::uint32_t session,
                   calibration::Micros absolute_deadline_us) override;
    bool endHold(motion::AxisId axis, std::uint32_t session) override;
    bool setRate(std::uint32_t session, motion::AxisId axis, float rate,
                 calibration::Micros lease_us) override;
    bool requestStop(std::uint64_t request_id) override;
    calibration::StopAttestation lastStopAttestation() const override;
    bool beginFlashWindow(calibration::Micros timeout_us) override;
    void endFlashWindow() override;
    calibration::Snapshot snapshot() const override;
    calibration::Micros nowUs() const override;

private:
    bool submitClaim(std::uint32_t session, bool claim, bool release);
    bool submitRates(std::uint32_t session, motion::AxisId axis, float rate,
                     calibration::Micros lease_us);
    bool waitFor(bool (*predicate)(void *), void *context,
                 calibration::Micros timeout_us);

    control_hal::ControlService &control_;
    state_hal::StateHub &hub_;
    std::uint32_t sequence_ = 0;
};

}  // namespace calibration_hal
