/**
 * @file fakes.hpp
 * @brief Тестовые реализации интерфейсов executor_io.hpp.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>

#include "motion/axis_command.hpp"
#include "motion/executor_io.hpp"

namespace fakes {

/// Часы и одноразовый alarm без реального времени.
class FakeClock final : public motion::AlarmSink {
public:
    std::uint64_t now = 0;
    bool armed = false;
    bool arm_fails = false;
    std::uint64_t deadline = 0;
    int arm_calls = 0;
    int disarm_calls = 0;

    bool armAt(std::uint64_t deadline_us) override
    {
        ++arm_calls;
        if (arm_fails) {
            return false;
        }
        armed = true;
        deadline = deadline_us;
        return true;
    }

    void disarm() override
    {
        ++disarm_calls;
        armed = false;
    }

    std::uint64_t nowUs() const override { return now; }

    bool due() const { return armed && now >= deadline; }
};

/// Фазы обмоток без GPIO.
class FakeCoil final : public motion::CoilSink {
public:
    int advances = 0;
    int releases = 0;
    int energizes = 0;
    bool energized = false;
    motion::Direction last_direction = motion::Direction::Forward;

    void advance(motion::Direction direction) override
    {
        ++advances;
        last_direction = direction;
        energized = true;
    }

    void release() override
    {
        ++releases;
        energized = false;
    }

    void energize() override
    {
        ++energizes;
        energized = true;
    }

    bool isEnergized() const override { return energized; }
};

/// Очередь команд без FreeRTOS.
class FakeSource final : public motion::CommandSource {
public:
    std::deque<motion::AxisCommand> queue;
    /// Ожидающий идентификатор приоритетного останова; 0 — запроса нет.
    std::uint64_t pending_emergency_id = 0;

    void push(const motion::AxisCommand &cmd) { queue.push_back(cmd); }

    std::size_t take(motion::AxisCommand *out, std::size_t max_count) override
    {
        std::size_t taken = 0;
        while (taken < max_count && !queue.empty()) {
            out[taken++] = queue.front();
            queue.pop_front();
        }
        return taken;
    }

    bool empty() const override { return queue.empty(); }

    void clear() override { queue.clear(); }

    std::uint64_t takeEmergencyId() override
    {
        const std::uint64_t taken = pending_emergency_id;
        pending_emergency_id = 0;
        return taken;
    }
};

}  // namespace fakes
