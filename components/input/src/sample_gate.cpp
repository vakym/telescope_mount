/**
 * @file sample_gate.cpp
 * @brief Последовательности ошибок и успехов чтения ADC.
 */

#include "input/sample_gate.hpp"

namespace input {

bool SampleGate::isValid(const SampleGateConfig &config) noexcept
{
    return config.error_threshold >= 1 && config.recovery_threshold >= 1;
}

void SampleGate::configure(const SampleGateConfig &config) noexcept
{
    config_ = isValid(config) ? config : SampleGateConfig{};
    reset();
}

void SampleGate::reset() noexcept
{
    errors_ = 0;
    successes_ = 0;
    faulted_ = false;
    output_valid_ = false;
}

SampleGateAction SampleGate::update(bool sample_valid) noexcept
{
    if (!sample_valid) {
        successes_ = 0;
        if (errors_ < config_.error_threshold) {
            ++errors_;
        }
        output_valid_ = false;
        if (errors_ >= config_.error_threshold) {
            faulted_ = true;
        }
        return SampleGateAction::Ignore;
    }

    errors_ = 0;
    if (!faulted_) {
        output_valid_ = true;
        return SampleGateAction::Process;
    }

    if (successes_ < config_.recovery_threshold) {
        ++successes_;
    }
    if (successes_ >= config_.recovery_threshold) {
        faulted_ = false;
        output_valid_ = true;
        return SampleGateAction::RecoverAndProcess;
    }

    output_valid_ = false;
    return SampleGateAction::Ignore;
}

}  // namespace input
