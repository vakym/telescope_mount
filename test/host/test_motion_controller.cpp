/**
 * @file test_motion_controller.cpp
 * @brief Независимость осей и проксирование команд.
 */

#include "check.hpp"
#include "motion/motion_controller.hpp"

namespace {

using motion::AxisCommand;
using motion::AxisId;
using motion::AxisState;
using motion::MotionController;

motion::AxisConfig baseConfig()
{
    return motion::AxisConfig{};
}

void axesAreIndependent()
{
    MotionController motion;
    CHECK(motion.configure(AxisId::Azimuth, baseConfig()));
    CHECK(motion.configure(AxisId::Altitude, baseConfig()));

    CHECK(motion.apply(AxisId::Azimuth, AxisCommand::enable()));
    CHECK(motion.apply(AxisId::Altitude, AxisCommand::enable()));

    CHECK(motion.apply(AxisId::Azimuth, AxisCommand::setTargetRate(400.0F)));
    CHECK(motion.wantsStep(AxisId::Azimuth));
    CHECK(!motion.wantsStep(AxisId::Altitude));
    CHECK_NEAR(motion.status(AxisId::Altitude).target_rate_steps_s, 0.0, 1e-6);
}

void disableBlocksMotion()
{
    MotionController motion;
    CHECK(motion.configure(AxisId::Azimuth, baseConfig()));

    CHECK(!motion.apply(AxisId::Azimuth, AxisCommand::setTargetRate(100.0F)));
    CHECK(motion.status(AxisId::Azimuth).rejected_commands == 1);

    CHECK(motion.apply(AxisId::Azimuth, AxisCommand::enable()));
    CHECK(motion.apply(AxisId::Azimuth, AxisCommand::setTargetRate(100.0F)));

    CHECK(motion.apply(AxisId::Azimuth, AxisCommand::disable()));
    CHECK(!motion.apply(AxisId::Azimuth, AxisCommand::setTargetRate(100.0F)));
    CHECK(motion.status(AxisId::Azimuth).state == AxisState::Disabled);
    CHECK(!motion.status(AxisId::Azimuth).position_trusted);
}

void stopAllInvalidatesPlans()
{
    MotionController motion;
    CHECK(motion.configure(AxisId::Azimuth, baseConfig()));
    CHECK(motion.configure(AxisId::Altitude, baseConfig()));
    CHECK(motion.apply(AxisId::Azimuth, AxisCommand::enable()));
    CHECK(motion.apply(AxisId::Altitude, AxisCommand::enable()));
    CHECK(motion.apply(AxisId::Azimuth, AxisCommand::moveSteps(10)));
    CHECK(motion.apply(AxisId::Altitude, AxisCommand::moveSteps(10)));

    motion::StepPlan plan{};
    CHECK(motion.prepareNextStep(AxisId::Azimuth, plan));
    CHECK(motion.prepareNextStep(AxisId::Altitude, plan));

    motion.apply(AxisId::Azimuth, AxisCommand::stop(motion::StopMode::Immediate));
    motion.apply(AxisId::Altitude, AxisCommand::stop(motion::StopMode::Immediate));

    CHECK(!motion.hasPending(AxisId::Azimuth));
    CHECK(!motion.hasPending(AxisId::Altitude));
    CHECK(motion.status(AxisId::Azimuth).state == AxisState::Released);
    CHECK(motion.status(AxisId::Altitude).state == AxisState::Released);
}

void statusAndPlanKey()
{
    MotionController motion;
    CHECK(motion.configure(AxisId::Azimuth, baseConfig()));
    CHECK(motion.apply(AxisId::Azimuth, AxisCommand::enable()));

    const std::uint32_t key = motion.planKey(AxisId::Azimuth);
    CHECK(motion.apply(AxisId::Azimuth, AxisCommand::setTargetRate(200.0F)));
    CHECK(motion.planKey(AxisId::Azimuth) != key);

    motion::StepPlan plan{};
    CHECK(motion.prepareNextStep(AxisId::Azimuth, plan));
    motion.commitPending(AxisId::Azimuth);
    CHECK(motion.status(AxisId::Azimuth).position_step == 1);

    motion.cancelPendingStep(AxisId::Azimuth);
    motion.forceReleased(AxisId::Azimuth, motion::ReleaseReason::Explicit);
    CHECK(motion.status(AxisId::Azimuth).state == AxisState::Released);
}

void invalidAxisIsRejected()
{
    MotionController motion;
    const auto invalid = static_cast<AxisId>(99);
    CHECK(!motion.configure(invalid, baseConfig()));
    CHECK(!motion.apply(invalid, AxisCommand::enable()));
    CHECK(!motion.hasPending(invalid));
    CHECK(motion.status(invalid).state == AxisState::Disabled);
}

void faultRequiresReset()
{
    MotionController motion;
    CHECK(motion.configure(AxisId::Altitude, baseConfig()));
    CHECK(motion.apply(AxisId::Altitude, AxisCommand::enable()));

    motion.enterFault(AxisId::Altitude, motion::FaultReason::DeadlineMiss);
    CHECK(motion.status(AxisId::Altitude).state == AxisState::Fault);
    CHECK(!motion.apply(AxisId::Altitude, AxisCommand::setTargetRate(50.0F)));

    CHECK(motion.apply(AxisId::Altitude, AxisCommand::resetFault()));
    CHECK(motion.status(AxisId::Altitude).state == AxisState::Disabled);
    CHECK(!motion.apply(AxisId::Altitude, AxisCommand::setTargetRate(50.0F)));
    CHECK(motion.apply(AxisId::Altitude, AxisCommand::enable()));
    CHECK(motion.apply(AxisId::Altitude, AxisCommand::setTargetRate(50.0F)));
}

}  // namespace

int main()
{
    axesAreIndependent();
    disableBlocksMotion();
    stopAllInvalidatesPlans();
    statusAndPlanKey();
    invalidAxisIsRejected();
    faultRequiresReset();
    return TEST_RESULT();
}
