/**
 * @file arbiter.hpp
 * @brief Владение парой осей: приоритеты, передача, lease.
 *
 * Реестр сессий и последовательностей живёт во входном канале
 * (IntentInbox/Arbiter вызываются только после проверки). Арбитр принимает
 * события владения и уставки раздельно, чтобы порядок «останов -> события ->
 * скорости» не разрушал проверку последовательностей.
 *
 * Приоритет фиксирован: Joystick > Browser > Automation.
 */

#pragma once

#include <cstdint>

#include "control/intent.hpp"
#include "state/channels.hpp"

namespace control {

class Arbiter {
public:
    struct Config {
        state::Micros default_lease_us = 200'000;
        state::Micros joystick_idle_release_us = 3'000'000;
    };

    void configure(const Config &config) noexcept;
    void reset() noexcept;

    /// Запрос/отдача владения. deny заполняется при отказе.
    void applyOwnership(SourceId source, bool claim, bool release,
                        std::uint32_t session, state::Micros now_us,
                        DenyReason &out_deny) noexcept;

    /// Уставки и lease источника; игнорируются, если сессия старее слотовой.
    void applyRates(SourceId source, const AxisRate (&axis)[2],
                    std::uint32_t session, state::Micros created_us,
                    state::Micros valid_until_us) noexcept;

    /// Снять истёкшие lease; потерявший владение помечается owner_expired.
    void expire(state::Micros now_us) noexcept;

    ArbitratedSetpoint arbitrate(state::Micros now_us) noexcept;

    bool revalidateClaim(state::Micros now_us, DenyReason &out_deny) const noexcept;
    void completeHandover(state::Micros now_us) noexcept;
    void cancelHandover() noexcept;
    void release(SourceId source, state::Micros now_us) noexcept;

    SourceId owner() const noexcept { return owner_; }
    std::uint32_t generation() const noexcept { return generation_; }
    bool handoverPending() const noexcept { return handover_pending_; }
    bool consumeOwnerExpired() noexcept;

    static int priorityRank(SourceId source) noexcept;

private:
    struct Slot {
        AxisRate axis[2]{};
        std::uint32_t session = 0;
        state::Micros valid_until_us = 0;
        bool occupied = false;
    };

    static bool validSource(SourceId source) noexcept;
    static std::size_t indexOf(SourceId source) noexcept;

    Config config_{};
    Slot slots_[state::kSourceCount]{};
    SourceId owner_ = SourceId::None;
    std::uint32_t generation_ = 0;
    SourceId pending_claim_ = SourceId::None;
    bool handover_pending_ = false;
    bool owner_expired_ = false;
};

}  // namespace control
