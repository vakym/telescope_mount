/**
 * @file intent_inbox.hpp
 * @brief Приём намерений: одна проверка, приём целиком, затем разделение.
 *
 * Намерение принимается атомарно: сначала проверяются session/sequence,
 * затем размещение обязательных событий, и только потом сообщение
 * раскладывается по каналам. Частичного приёма нет.
 *
 * После приёма каналы разделены:
 *  - скорости  — latest-value на источник;
 *  - останов   — защёлка с полными сведениями, снимается один раз;
 *  - владение  — очередь событий, порядок обязателен.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "control/intent.hpp"
#include "state/channels.hpp"

namespace control {

class IntentInbox {
public:
    static constexpr std::size_t kEventCapacity = 4;

    enum class EventKind : std::uint8_t { Claim, Release };

    struct Event {
        SourceId source = SourceId::None;
        EventKind kind = EventKind::Claim;
        std::uint32_t session = 0;
        std::uint32_t sequence = 0;
        Micros created_us = 0;
    };

    struct StopRequest {
        SourceId source = SourceId::None;
        std::uint32_t session = 0;
        std::uint32_t sequence = 0;
        Micros created_us = 0;
    };

    /// false — сообщение отклонено целиком, состояние не изменено.
    bool submit(const Intent &intent, Micros now_us, DenyReason &out_deny) noexcept;

    std::size_t drainRates(Intent *out, std::size_t max_count) noexcept;
    std::size_t drainEvents(Event *out, std::size_t max_count) noexcept;

    /// Полные сведения о защёлкнутом запросе останова; снимает защёлку.
    bool takeStop(StopRequest &out) noexcept;
    bool stopLatched() const noexcept { return stop_latched_; }

    void clear() noexcept;

private:
    static bool validSource(SourceId source) noexcept;
    static std::size_t indexOf(SourceId source) noexcept;
    std::size_t eventCount() const noexcept;

    Intent rates_[state::kSourceCount]{};
    bool rate_occupied_[state::kSourceCount]{};
    Event events_[kEventCapacity]{};
    std::size_t head_ = 0;
    std::size_t count_ = 0;
    bool stop_latched_ = false;
    StopRequest stop_{};
    std::uint32_t last_session_[state::kSourceCount]{};
    std::uint32_t last_sequence_[state::kSourceCount]{};
    bool seen_[state::kSourceCount]{};
};

}  // namespace control
