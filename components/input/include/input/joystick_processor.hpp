/**
 * @file joystick_processor.hpp
 * @brief Калибровка центра и обработка положений стика.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "input/joystick_config.hpp"

namespace input {

/**
 * @brief Оценка центра по стабильной серии отсчётов.
 *
 * Отсчёты должны приходить по одному за период опроса, а не подряд:
 * проверяется стабильность во времени, а не быстрое чтение.
 */
class CenterCalibrator {
public:
    /**
     * @brief Настраивает серию.
     *
     * @return false, если samples == 0, noise_limit не конечен или
     * отрицателен, либо границы центра не конечны или не упорядочены.
     */
    bool configure(std::size_t samples, float noise_limit,
                   float center_min, float center_max) noexcept;

    void reset() noexcept;

    /// true = валидный отсчёт принят и сбор продолжается.
    bool add(const RawSample &sample) noexcept;

    bool ready() const noexcept;
    bool stable() const noexcept;
    float center(StickAxis axis) const noexcept;

private:
    bool withinBounds() const noexcept;
    bool quietEnough() const noexcept;

    std::size_t samples_ = 0;
    std::size_t count_ = 0;
    float noise_limit_ = 0.0F;
    float center_min_ = 0.0F;
    float center_max_ = 0.0F;
    double sum_[kAxisCount] = {0.0, 0.0};
    std::uint16_t min_[kAxisCount] = {0, 0};
    std::uint16_t max_[kAxisCount] = {0, 0};
    bool configured_ = false;
    bool have_sample_ = false;
    bool complete_ = false;
    bool stable_ = false;
};

/**
 * @brief Фильтрация, нормализация и радиальная мёртвая зона.
 *
 * Класс не зависит от времени и не знает о кнопке.
 */
class JoystickProcessor {
public:
    bool configure(const JoystickConfig &config) noexcept;

    /// Возвращает фильтр в центр и сбрасывает историю гистерезиса.
    void reset() noexcept;

    /**
     * @brief Полный детерминированный сброс по свежему валидному отсчёту.
     *
     * Инициализирует фильтр, сбрасывает историю гистерезиса в нейтраль,
     * сразу рассчитывает нормализацию, magnitude и состояние по порогу
     * dead_zone. Дополнительный update() тем же отсчётом не требуется.
     */
    StickState resetWithSample(const RawSample &sample) noexcept;

    /// Обновляет состояние; невалидный отсчёт не меняет его.
    StickState update(const RawSample &sample) noexcept;

    const JoystickConfig &config() const noexcept { return config_; }
    const StickState &state() const noexcept { return current_; }

private:
    StickState makeState(bool apply_hysteresis) noexcept;
    float normalize(StickAxis axis) const noexcept;

    JoystickConfig config_{};
    bool configured_ = false;
    bool neutral_ = true;
    float filtered_[kAxisCount] = {0.0F, 0.0F};
    StickState current_{};
};

}  // namespace input
