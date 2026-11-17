/**
 * @file control_port.cpp
 * @brief Реализация адаптера калибровки над ControlService.
 */

#include "calibration_hal/control_port.hpp"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

namespace calibration_hal {

ControlPort::ControlPort(control_hal::ControlService &control,
                         state_hal::StateHub &hub) noexcept
    : control_(control), hub_(hub)
{
}

bool ControlPort::waitFor(bool (*predicate)(void *), void *context,
                          calibration::Micros timeout_us)
{
    const calibration::Micros deadline = nowUs() + timeout_us;
    while (nowUs() < deadline) {
        if (predicate(context)) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return predicate(context);
}

bool ControlPort::submitClaim(std::uint32_t session, bool claim, bool release)
{
    control::Intent intent{};
    intent.source = state::SourceId::Calibration;
    intent.session = session;
    intent.sequence = ++sequence_;
    intent.created_us = nowUs();
    intent.valid_until_us = intent.created_us + CONFIG_CAL_READINESS_LEASE_MS * 1000ULL;
    intent.claim = claim;
    intent.release = release;
    // Нулевые уставки: владение само по себе не разрешает вращение.
    intent.axis[0].valid = true;
    intent.axis[1].valid = true;
    intent.axis[0].half_steps_per_s = 0.0F;
    intent.axis[1].half_steps_per_s = 0.0F;
    return control_.submitIntent(intent) == ESP_OK;
}

bool ControlPort::claim(std::uint32_t session)
{
    if (session == 0U) {
        return false;
    }
    return submitClaim(session, true, false);
}

void ControlPort::release(std::uint32_t session)
{
    if (session == 0U) {
        return;
    }
    (void)submitClaim(session, false, true);
}

void ControlPort::setInputReady(bool ready, std::uint32_t session,
                                calibration::Micros valid_until_us)
{
    control_.setCalibrationInputReady(ready, session, valid_until_us);
}

bool ControlPort::beginHold(motion::AxisId axis, std::uint32_t session,
                            calibration::Micros absolute_deadline_us)
{
    if (control_.beginCalibrationHold(axis, session, absolute_deadline_us) != ESP_OK) {
        return false;
    }
    struct Probe {
        control_hal::ControlService *control;
        motion::AxisId axis;
        std::uint32_t session;
    } probe{&control_, axis, session};
    return waitFor(
        [](void *raw) {
            auto *p = static_cast<Probe *>(raw);
            return p->control->calibrationHoldActive(p->axis, p->session);
        },
        &probe, 500'000);
}

bool ControlPort::endHold(motion::AxisId axis, std::uint32_t session)
{
    return control_.endCalibrationHold(axis, session) == ESP_OK;
}

bool ControlPort::holdActive(motion::AxisId axis, std::uint32_t session) const
{
    return control_.calibrationHoldActive(axis, session);
}

bool ControlPort::submitRates(std::uint32_t session, motion::AxisId axis, float rate,
                              calibration::Micros lease_us)
{
    control::Intent intent{};
    intent.source = state::SourceId::Calibration;
    intent.session = session;
    intent.sequence = ++sequence_;
    intent.created_us = nowUs();
    // Срок уставки задаёт вызывающий: продлевать его за дедлайн разрешения
    // браузера нельзя. Нулевой срок означает «до следующего цикла».
    intent.valid_until_us = intent.created_us + (lease_us == 0U ? 1'000U : lease_us);
    intent.claim = true;
    const std::size_t index = static_cast<std::size_t>(axis);
    if (index >= 2) {
        return false;
    }
    intent.axis[0].valid = true;
    intent.axis[1].valid = true;
    intent.axis[0].half_steps_per_s = 0.0F;
    intent.axis[1].half_steps_per_s = 0.0F;
    intent.axis[index].half_steps_per_s = rate;
    return control_.submitIntent(intent) == ESP_OK;
}

bool ControlPort::setRate(std::uint32_t session, motion::AxisId axis, float rate,
                          calibration::Micros lease_us)
{
    return submitRates(session, axis, rate, lease_us);
}

bool ControlPort::requestStop(std::uint64_t request_id)
{
    return control_.requestCalibrationStop(request_id) == ESP_OK;
}

calibration::StopAttestation ControlPort::lastStopAttestation() const
{
    const control_hal::CalibrationStopAttestation source = control_.lastStopAttestation();
    calibration::StopAttestation out{};
    out.valid = source.valid;
    out.request_id = source.request_id;
    out.requested_us = source.requested_us;
    out.confirmed_us = source.confirmed_us;
    out.immediate = source.immediate;
    out.confirmed = source.confirmed;
    out.motion_generation = source.motion_generation;
    for (std::size_t i = 0; i < 2; ++i) {
        out.ticket_id[i] = source.ticket_id[i];
        out.epoch[i] = source.epoch[i];
        out.position_step[i] = source.position_step[i];
        out.polled_us[i] = source.polled_us[i];
        out.position_trusted[i] = source.position_trusted[i];
        out.energized[i] = source.energized[i];
    }
    return out;
}

bool ControlPort::beginFlashWindow(calibration::Micros timeout_us)
{
    if (control_.beginFlashWindow(timeout_us) != ESP_OK) {
        return false;
    }
    struct Probe {
        control_hal::ControlService *control;
    } probe{&control_};
    return waitFor(
        [](void *raw) {
            auto *p = static_cast<Probe *>(raw);
            return p->control->flashWindowActive();
        },
        &probe, 6'000'000);
}

void ControlPort::endFlashWindow()
{
    control_.endFlashWindow();
}

calibration::Snapshot ControlPort::snapshot() const
{
    const state::SystemSnapshot hub = hub_.snapshot();
    const control_hal::ControlStatus status = control_.status();

    calibration::Snapshot out{};
    for (std::size_t i = 0; i < 2; ++i) {
        out.position_step[i] = hub.axis[i].status.position_step;
        out.position_trusted[i] = hub.axis[i].meta.valid &&
                                  hub.axis[i].status.position_trusted;
        out.has_fault[i] = hub.axis[i].status.has_fault;
        out.energized[i] = hub.axis[i].status.energized;
        out.state[i] = hub.axis[i].status.state;
        out.command_epoch[i] = hub.axis[i].status.command_epoch;
        out.rate[i] = hub.axis[i].status.rate_steps_s;
    }
    out.owner = status.owner;
    out.phase = status.phase;
    out.permit = status.permit;
    out.owner_generation = status.owner_generation;
    out.motion_generation = control_.motionGeneration();
    out.input_ready = control_.calibrationInputReady();
    out.flash_window_active = control_.flashWindowActive();
    return out;
}

calibration::Micros ControlPort::nowUs() const
{
    return static_cast<calibration::Micros>(esp_timer_get_time());
}

}  // namespace calibration_hal
