/**
 * @file input_grace.hpp
 * @brief Ограниченное удержание последней уставки при единичной ошибке ADC.
 *
 * Grace разрешён только для кратковременной ошибки отсчёта. Подтверждённый
 * fault, устаревание, неверная конфигурация, потеря разрешения, handover и
 * моторный Fault его запрещают. Срок отсчитывается от ПЕРВОЙ ошибки и не
 * продлевается повторными; он также не может превышать допустимый возраст
 * последнего хорошего измерения.
 */

#pragma once

#include "state/channels.hpp"
#include "state/observation.hpp"

namespace control {

class InputGracePolicy {
public:
    struct Config {
        state::Micros grace_us = 100'000;
    };

    struct Decision {
        bool hold_previous = false;  ///< разрешено удержать ранее принятую уставку
        bool expired = false;        ///< удержание недоступно
    };

    /// false при нулевом grace или нулевом допустимом возрасте измерения.
    bool configure(const Config &config, state::Micros stale_after_us) noexcept;

    void onGoodSample(state::Micros measured_us) noexcept;

    /// Первая ошибка запускает срок; повторные его НЕ продлевают.
    Decision onRejected(state::DenyReason reason, state::Micros now_us) noexcept;

    state::Micros deadline() const noexcept;
    bool active() const noexcept { return active_; }
    void reset() noexcept;

private:
    Config config_{};
    state::Micros stale_after_us_ = 0;
    state::Micros started_us_ = 0;
    state::Micros last_good_us_ = 0;
    bool active_ = false;
    bool have_good_ = false;
};

}  // namespace control
