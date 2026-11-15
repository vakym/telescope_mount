/**
 * @file test_axis_command_ack.cpp
 * @brief Подтверждение команд исполнителем: тикеты и канал emergency-id.
 */

#include <cstddef>
#include <cstdint>
#include <deque>

#include "check.hpp"
#include "fakes.hpp"
#include "motion/axis_executor.hpp"
#include "motion/axis_planner.hpp"

namespace {

class Source final : public motion::CommandSource {
public:
    std::deque<motion::AxisCommand> queue;
    std::uint64_t pending_emergency = 0;
    std::uint64_t last_taken = 0;

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
        last_taken = pending_emergency;
        pending_emergency = 0;
        return last_taken;
    }
};

struct Fixture {
    motion::AxisPlanner planner;
    fakes::FakeClock clock;
    fakes::FakeCoil coil;
    Source source;
    motion::AxisExecutor executor;

    Fixture()
        : executor(planner, clock, coil, source,
                   motion::AxisExecutor::Params{5000, 8, 3})
    {
        motion::AxisConfig config{};
        config.id = motion::AxisId::Azimuth;
        (void)planner.configure(config);
    }
};

void commandIdIsTracked()
{
    Fixture fixture;
    motion::AxisCommand enable = motion::AxisCommand::enable();
    enable.id = 5;
    fixture.source.push(enable);

    fixture.executor.onWakeup(motion::CommandPending);
    const motion::AxisStatus status = fixture.executor.status();
    CHECK(status.last_applied_command_id == 5U);
    CHECK(status.command_epoch == 3U);
}

void zeroIdDoesNotOverwrite()
{
    Fixture fixture;
    motion::AxisCommand enable = motion::AxisCommand::enable();
    enable.id = 5;
    fixture.source.push(enable);
    fixture.executor.onWakeup(motion::CommandPending);

    fixture.source.push(motion::AxisCommand::setTargetRate(10.0F));  // id = 0
    fixture.executor.onWakeup(motion::CommandPending);
    CHECK(fixture.executor.status().last_applied_command_id == 5U);
}

void emergencyAcksTakenIdOnly()
{
    Fixture fixture;
    motion::AxisCommand enable = motion::AxisCommand::enable();
    enable.id = 5;
    fixture.source.push(enable);
    fixture.executor.onWakeup(motion::CommandPending);

    // В очереди остаётся команда, которую emergency должен погасить.
    motion::AxisCommand rate = motion::AxisCommand::setTargetRate(200.0F);
    rate.id = 6;
    fixture.source.push(rate);

    fixture.source.pending_emergency = 9;
    fixture.executor.onWakeup(motion::EmergencyStop);

    const motion::AxisStatus status = fixture.executor.status();
    CHECK(status.last_applied_command_id == 9U);
    CHECK(fixture.source.empty());
    CHECK(fixture.coil.releases > 0);
}

void unmatchedBitDoesNotClearQueue()
{
    Fixture fixture;
    fixture.source.pending_emergency = 20;
    fixture.executor.onWakeup(motion::EmergencyStop);
    CHECK(fixture.executor.status().last_applied_command_id == 20U);
    const std::uint32_t unmatched_before =
        fixture.executor.status().unmatched_emergency_notifications;

    motion::AxisCommand rate = motion::AxisCommand::setTargetRate(50.0F);
    rate.id = 21;
    fixture.source.push(rate);

    // Бит без отслеживаемого id и без TaskExit: очередь не очищается.
    fixture.executor.onWakeup(motion::EmergencyStop);
    CHECK(!fixture.source.empty());
    CHECK(fixture.executor.status().last_applied_command_id == 20U);
    CHECK(fixture.executor.status().unmatched_emergency_notifications ==
          unmatched_before + 1U);
}

void taskExitStopsWithoutTicket()
{
    Fixture fixture;
    fixture.source.pending_emergency = 30;
    fixture.executor.onWakeup(motion::EmergencyStop);
    CHECK(fixture.executor.status().last_applied_command_id == 30U);

    motion::AxisCommand rate = motion::AxisCommand::setTargetRate(50.0F);
    rate.id = 31;
    fixture.source.push(rate);

    fixture.executor.onWakeup(motion::EmergencyStop | motion::TaskExit);
    CHECK(fixture.source.empty());
    // Служебный путь подтверждается завершением задачи, а не тикетом.
    CHECK(fixture.executor.status().last_applied_command_id == 30U);
}

void secondEmergencyAfterAck()
{
    Fixture fixture;
    fixture.source.pending_emergency = 40;
    fixture.executor.onWakeup(motion::EmergencyStop);
    CHECK(fixture.executor.status().last_applied_command_id == 40U);

    fixture.source.pending_emergency = 41;
    fixture.executor.onWakeup(motion::EmergencyStop);
    CHECK(fixture.executor.status().last_applied_command_id == 41U);
}

}  // namespace

int main()
{
    commandIdIsTracked();
    zeroIdDoesNotOverwrite();
    emergencyAcksTakenIdOnly();
    unmatchedBitDoesNotClearQueue();
    taskExitStopsWithoutTicket();
    secondEmergencyAfterAck();
    return TEST_RESULT();
}
