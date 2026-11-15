/**
 * @file sample_gate.hpp
 * @brief Политика последовательных ошибок и успехов чтения ADC.
 *
 * Класс не работает со временем и не знает о кнопке. Счётчики насыщаются
 * на порогах, поэтому не растут бесконечно.
 */

#pragma once

#include <cstdint>

namespace input {

enum class SampleGateAction : std::uint8_t {
    Ignore,             ///< положение не обновлять
    Process,            ///< обычное обновление
    RecoverAndProcess,  ///< снять fault и переинициализировать фильтр
};

struct SampleGateConfig {
    std::uint32_t error_threshold = 5;
    std::uint32_t recovery_threshold = 5;
};

class SampleGate {
public:
    /// true, если оба порога не меньше 1.
    bool isValid(const SampleGateConfig &config) noexcept;

    /// false = конфигурация отвергнута; значения по умолчанию не подставляются.
    bool configure(const SampleGateConfig &config) noexcept;

    void reset() noexcept;

    SampleGateAction update(bool sample_valid) noexcept;

    bool outputValid() const noexcept { return output_valid_; }
    bool faulted() const noexcept { return faulted_; }
    std::uint32_t consecutiveErrors() const noexcept { return errors_; }
    std::uint32_t consecutiveSuccesses() const noexcept { return successes_; }

private:
    SampleGateConfig config_{};
    std::uint32_t errors_ = 0;
    std::uint32_t successes_ = 0;
    bool faulted_ = false;
    bool output_valid_ = false;
};

}  // namespace input
