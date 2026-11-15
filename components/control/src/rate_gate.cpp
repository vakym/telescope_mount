/**
 * @file rate_gate.cpp
 * @brief Реализация дедупликации уставок.
 */

#include "control/rate_gate.hpp"

#include <cmath>
#include <cstddef>

namespace control {

std::size_t RateGate::indexOf(motion::AxisId id) noexcept
{
    return static_cast<std::size_t>(id);
}

void RateGate::configure(const Config &config) noexcept
{
    config_ = config;
    if (!(config_.deadband > 0.0F)) {
        config_.deadband = 0.0F;  // нулевая полоса допустима: точное сравнение
    }
    reset();
}

void RateGate::reset() noexcept
{
    resetSlots();
    generation_ = 0;
}

void RateGate::resetSlots() noexcept
{
    // Поколение общее для гейта, поэтому при его смене сбрасываются ОБА слота:
    // иначе вторая ось сочла бы прежнюю уставку дублем и осталась стоять.
    slots_[0] = Slot{};
    slots_[1] = Slot{};
}

RateGate::Decision RateGate::evaluate(motion::AxisId id, float desired,
                                      std::uint32_t generation) noexcept
{
    Decision decision{};
    if (id != motion::AxisId::Azimuth && id != motion::AxisId::Altitude) {
        return decision;
    }
    // 1. Смена поколения уничтожает недоставленное прежнего владельца
    //    по обеим осям.
    if (generation != generation_) {
        generation_ = generation;
        resetSlots();
    }
    Slot &slot = slots_[indexOf(id)];

    if (!std::isfinite(desired)) {
        return decision;
    }

    const float value = desired;

    // 2. Нуль и останов вытесняют ожидающую ненулевую уставку.
    if (value == 0.0F) {
        if (!slot.known || slot.value != 0.0F || slot.pending) {
            decision.send = true;
            decision.value = 0.0F;
        }
        return decision;
    }

    // 3. Ненулевая уставка: первая, смена знака, значимое изменение или
    //    повтор после отказа доставки — отправляем АКТУАЛЬНОЕ значение.
    const bool sign_changed = slot.known && ((slot.value > 0.0F) != (value > 0.0F));
    const bool beyond_deadband =
        slot.known && std::fabs(value - slot.value) >= config_.deadband;
    if (!slot.known || sign_changed || beyond_deadband || slot.pending) {
        decision.send = true;
        decision.value = value;
    }
    return decision;
}

void RateGate::noteEnqueued(motion::AxisId id, float value,
                            std::uint32_t generation) noexcept
{
    if (id != motion::AxisId::Azimuth && id != motion::AxisId::Altitude) {
        return;
    }
    if (generation != generation_) {
        // Команда другого поколения не должна оставлять старый слот второй оси.
        generation_ = generation;
        resetSlots();
    }
    Slot &slot = slots_[indexOf(id)];
    slot.known = true;
    slot.pending = false;
    slot.value = value;
}

void RateGate::noteFailed(motion::AxisId id) noexcept
{
    if (id != motion::AxisId::Azimuth && id != motion::AxisId::Altitude) {
        return;
    }
    // Недоставленная команда не считается отправленной.
    slots_[indexOf(id)].pending = true;
}

bool RateGate::hasEnqueuedNonZero(motion::AxisId id) const noexcept
{
    if (id != motion::AxisId::Azimuth && id != motion::AxisId::Altitude) {
        return false;
    }
    const Slot &slot = slots_[indexOf(id)];
    return slot.known && slot.value != 0.0F;
}

float RateGate::lastEnqueued(motion::AxisId id) const noexcept
{
    if (id != motion::AxisId::Azimuth && id != motion::AxisId::Altitude) {
        return 0.0F;
    }
    return slots_[indexOf(id)].value;
}

bool RateGate::pending(motion::AxisId id) const noexcept
{
    if (id != motion::AxisId::Azimuth && id != motion::AxisId::Altitude) {
        return false;
    }
    return slots_[indexOf(id)].pending;
}

}  // namespace control
