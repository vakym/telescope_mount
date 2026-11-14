/**
 * @file coil_driver.hpp
 * @brief Управление четырьмя линиями ULN2003 через motion::CoilSink.
 */

#pragma once

#include <cstdint>

#include "board/board.hpp"
#include "esp_err.h"
#include "motion/executor_io.hpp"
#include "motion/types.hpp"

namespace motor_hal {

class CoilDriver final : public motion::CoilSink {
public:
    /**
     * @brief Настраивает линии и инициализирует индекс фазы.
     *
     * Первым действием все линии переводятся в выход с низким уровнем.
     */
    esp_err_t init(const board::CoilPins &pins, motion::StepPattern pattern, bool invert);

    void advance(motion::Direction direction) override;
    void release() override;
    void energize() override;
    bool isEnergized() const override { return energized_; }

private:
    void applyPhase();

    board::CoilPins pins_{};
    motion::StepPattern pattern_ = motion::StepPattern::Half8;
    bool invert_ = false;
    std::uint8_t index_ = 0;
    bool energized_ = false;
};

}  // namespace motor_hal
