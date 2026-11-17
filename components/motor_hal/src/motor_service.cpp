/**
 * @file motor_service.cpp
 * @brief Сборка двух осей и проксирование команд.
 */

#include "motor_hal/motor_service.hpp"

#include "esp_check.h"
#include "esp_log.h"

namespace motor_hal {
namespace {

constexpr const char *kTag = "motor_service";

}  // namespace

bool MotorService::valid(motion::AxisId id) noexcept
{
    return static_cast<std::uint8_t>(id) < static_cast<std::uint8_t>(motion::AxisId::Count);
}

esp_err_t MotorService::start(const board::MotorPins &pins, const motion::AxisConfig (&configs)[2])
{
    if (state_ == State::Running) {
        return ESP_ERR_INVALID_STATE;
    }

    // После неудачного stop() оси могли остаться в Stopping: повторная
    // конфигурация планировщиков недопустима, пока они не завершились.
    const esp_err_t stop_err = stop();
    if (stop_err != ESP_OK) {
        return stop_err;
    }

    for (auto &motor : motors_) {
        const esp_err_t err = motor.stop();
        if (err != ESP_OK) {
            state_ = State::Stopping;
            return err;
        }
    }

    for (std::size_t i = 0; i < kAxisCount; ++i) {
        const auto id = static_cast<motion::AxisId>(i);
        ESP_RETURN_ON_FALSE(motion_.configure(id, configs[i]), ESP_ERR_INVALID_ARG,
                            kTag, "invalid axis config for axis %u", static_cast<unsigned>(i));
        hold_policy_[i] = configs[i].hold;
    }

    const esp_err_t azimuth_err = motors_[0].start(motion_.planner(motion::AxisId::Azimuth),
                                                   pins.azimuth, configs[0]);
    if (azimuth_err != ESP_OK) {
        state_ = State::Idle;  // start() оси откатил свои ресурсы
        return azimuth_err;
    }

    const esp_err_t altitude_err = motors_[1].start(motion_.planner(motion::AxisId::Altitude),
                                                    pins.altitude, configs[1]);
    if (altitude_err != ESP_OK) {
        // Откат частичной инициализации: первая ось не должна остаться живой.
        ESP_LOGE(kTag, "altitude start failed, stopping azimuth");
        const esp_err_t rollback_err = stop();
        if (rollback_err != ESP_OK) {
            state_ = State::Stopping;
            return rollback_err;
        }
        return altitude_err;
    }

    state_ = State::Running;
    return ESP_OK;
}

esp_err_t MotorService::stop()
{
    esp_err_t result = ESP_OK;
    // Останавливаем обе оси всегда, даже если первая вернула ошибку.
    for (auto &motor : motors_) {
        const esp_err_t err = motor.stop();
        if (result == ESP_OK && err != ESP_OK) {
            result = err;
        }
    }

    state_ = (result == ESP_OK) ? State::Idle : State::Stopping;
    return result;
}

CommandReceipt MotorService::enable(motion::AxisId id)
{
    if (!valid(id)) {
        return CommandReceipt{};
    }
    return motors_[static_cast<std::size_t>(id)].post(motion::AxisCommand::enable());
}

bool MotorService::disable(motion::AxisId id)
{
    return valid(id) &&
           motors_[static_cast<std::size_t>(id)].post(motion::AxisCommand::disable()).accepted;
}

CommandReceipt MotorService::setTargetRate(motion::AxisId id, float half_steps_per_s)
{
    if (!valid(id)) {
        return CommandReceipt{};
    }
    return motors_[static_cast<std::size_t>(id)].post(
        motion::AxisCommand::setTargetRate(half_steps_per_s));
}

bool MotorService::moveSteps(motion::AxisId id, std::int64_t half_steps)
{
    return valid(id) &&
           motors_[static_cast<std::size_t>(id)].post(motion::AxisCommand::moveSteps(half_steps))
               .accepted;
}

CommandReceipt MotorService::stopAxis(motion::AxisId id, motion::StopMode mode)
{
    if (!valid(id)) {
        return CommandReceipt{};
    }
    if (mode == motion::StopMode::Immediate) {
        return motors_[static_cast<std::size_t>(id)].emergencyStop();
    }
    return motors_[static_cast<std::size_t>(id)].post(motion::AxisCommand::stop(mode));
}

PairReceipt MotorService::stopAll(motion::StopMode mode)
{
    PairReceipt receipts{};
    if (mode == motion::StopMode::Immediate) {
        receipts.azimuth = motors_[0].emergencyStop();
        receipts.altitude = motors_[1].emergencyStop();
        return receipts;
    }
    receipts.azimuth = motors_[0].post(motion::AxisCommand::stop(mode));
    receipts.altitude = motors_[1].post(motion::AxisCommand::stop(mode));
    return receipts;
}

bool MotorService::release(motion::AxisId id)
{
    return valid(id) &&
           motors_[static_cast<std::size_t>(id)].post(motion::AxisCommand::release()).accepted;
}

bool MotorService::energize(motion::AxisId id)
{
    return valid(id) &&
           motors_[static_cast<std::size_t>(id)].post(motion::AxisCommand::energize()).accepted;
}

bool MotorService::setHoldPolicy(motion::AxisId id, const motion::HoldPolicy &policy)
{
    if (!valid(id)) {
        return false;
    }
    const bool accepted =
        motors_[static_cast<std::size_t>(id)]
            .post(motion::AxisCommand::setHoldPolicy(policy))
            .accepted;
    if (accepted) {
        hold_policy_[static_cast<std::size_t>(id)] = policy;
    }
    return accepted;
}

motion::HoldPolicy MotorService::holdPolicy(motion::AxisId id) const
{
    if (!valid(id)) {
        return motion::HoldPolicy{};
    }
    return hold_policy_[static_cast<std::size_t>(id)];
}

CommandReceipt MotorService::resetFault(motion::AxisId id)
{
    if (!valid(id)) {
        return CommandReceipt{};
    }
    return motors_[static_cast<std::size_t>(id)].post(motion::AxisCommand::resetFault());
}

std::uint32_t MotorService::emergencySlotBusy() const noexcept
{
    return motors_[0].emergencySlotBusy() + motors_[1].emergencySlotBusy();
}

std::uint32_t MotorService::notifyFailures() const noexcept
{
    return motors_[0].notifyFailures() + motors_[1].notifyFailures();
}

motion::AxisStatus MotorService::status(motion::AxisId id) const
{
    if (!valid(id)) {
        return motion::AxisStatus{};
    }
    return motors_[static_cast<std::size_t>(id)].status();
}

}  // namespace motor_hal
