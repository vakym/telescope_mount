/**
 * @file state_hub.hpp
 * @brief Единственный владелец наблюдаемых снимков и их публикация.
 *
 * Хаб — зеркало, а не источник истины: он ничего не читает у сервисов и не
 * принимает решений. Под мьютексом только копирование POD: без вызовов HAL,
 * логирования, ожиданий и callback.
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "motion/axis_planner.hpp"
#include "state/observation.hpp"

namespace state_hal {

class StateHub {
public:
    StateHub();
    ~StateHub() = default;

    StateHub(const StateHub &) = delete;
    StateHub &operator=(const StateHub &) = delete;

    /// Копия статуса исполнителя. published_us и sequence назначает хаб.
    void publishAxis(motion::AxisId id, const motion::AxisStatus &status,
                     state::Micros polled_us, std::uint32_t read_failures) noexcept;

    /// Полная замена блока джойстика. measured_us переносится как есть.
    void publishJoystick(const state::JoystickObservation &obs) noexcept;

    /// Состояние контура; пишет только управляющая задача.
    void publishControl(const state::ControlObservation &obs) noexcept;

    /// Пометить блок недействительным, не подменяя время измерения.
    void invalidateAxis(motion::AxisId id) noexcept;
    void invalidateJoystick() noexcept;

    /// Один захват мьютекса, копия всех блоков.
    state::SystemSnapshot snapshot() const noexcept;

private:
    static bool valid(motion::AxisId id) noexcept;

    mutable StaticSemaphore_t mutex_storage_{};
    SemaphoreHandle_t mutex_ = nullptr;
    state::SystemSnapshot published_{};
};

}  // namespace state_hal
