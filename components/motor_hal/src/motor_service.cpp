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

bool MotorService::enable(motion::AxisId id)
{
    return valid(id) && motors_[static_cast<std::size_t>(id)].post(motion::AxisCommand::enable());
}

bool MotorService::disable(motion::AxisId id)
{
    return valid(id) && motors_[static_cast<std::size_t>(id)].post(motion::AxisCommand::disable());
}

bool MotorService::setTargetRate(motion::AxisId id, float half_steps_per_s)
{
    return valid(id) && motors_[static_cast<std::size_t>(id)].post(
                            motion::AxisCommand::setTargetRate(half_steps_per_s));
}

bool MotorService::moveSteps(motion::AxisId id, std::int64_t half_steps)
{
    return valid(id) && motors_[static_cast<std::size_t>(id)].post(
                            motion::AxisCommand::moveSteps(half_steps));
}

bool MotorService::stopAxis(motion::AxisId id, motion::StopMode mode)
{
    if (!valid(id)) {
        return false;
    }
    if (mode == motion::StopMode::Immediate) {
        return motors_[static_cast<std::size_t>(id)].emergencyStop();
    }
    return motors_[static_cast<std::size_t>(id)].post(motion::AxisCommand::stop(mode));
}

bool MotorService::stopAll(motion::StopMode mode)
{
    bool ok = true;
    for (auto &motor : motors_) {
        if (mode == motion::StopMode::Immediate) {
            ok = motor.emergencyStop() && ok;
        } else {
            ok = motor.post(motion::AxisCommand::stop(mode)) && ok;
        }
    }
    return ok;
}

bool MotorService::release(motion::AxisId id)
{
    return valid(id) && motors_[static_cast<std::size_t>(id)].post(motion::AxisCommand::release());
}

bool MotorService::energize(motion::AxisId id)
{
    return valid(id) && motors_[static_cast<std::size_t>(id)].post(motion::AxisCommand::energize());
}

bool MotorService::setHoldPolicy(motion::AxisId id, const motion::HoldPolicy &policy)
{
    return valid(id) && motors_[static_cast<std::size_t>(id)].post(
                            motion::AxisCommand::setHoldPolicy(policy));
}

bool MotorService::resetFault(motion::AxisId id)
{
    return valid(id) && motors_[static_cast<std::size_t>(id)].post(
                            motion::AxisCommand::resetFault());
}

motion::AxisStatus MotorService::status(motion::AxisId id) const
{
    if (!valid(id)) {
        return motion::AxisStatus{};
    }
    return motors_[static_cast<std::size_t>(id)].status();
}

}  // namespace motor_hal
