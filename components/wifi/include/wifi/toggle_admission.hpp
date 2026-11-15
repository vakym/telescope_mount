/**
 * @file toggle_admission.hpp
 * @brief Единственный владелец количества принятых заявок на переключение.
 *
 * Приём заявки — одна согласованная операция: проверка вместимости и
 * резервирование через compare_exchange не разрываются. Координатор
 * операций о заявках не знает; служба имеет ровно один экземпляр этого
 * класса.
 *
 * Учёт результатов раздельный:
 *  - `failed_on_*` относится к уже извлечённой заявке;
 *  - `dropped_on_*` относится только к ожидающим заявкам.
 * Одна заявка никогда не учитывается одновременно в двух группах, поэтому
 * после завершения обработки выполняется баланс
 * `accepted == executed + failed_on_* + dropped_on_*`.
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "wifi/ap_lifecycle.hpp"

namespace wifi {

constexpr std::uint8_t kMaxPendingToggles = 2;

class ToggleAdmission {
public:
    enum class Result : std::uint8_t {
        Accepted,
        NotRunning,
        Unconfirmed,
        Full,
    };

    enum class DropReason : std::uint8_t {
        Shutdown,
        Timeout,
        Error,
    };

    struct Outcome {
        Result result = Result::NotRunning;
        /// true для Accepted и Unconfirmed: в обоих случаях задача должна
        /// проснуться. Для Unconfirmed это повод для ресинхронизации.
        bool wake_task = false;
    };

    /// Проверка и резервирование — одна согласованная операция.
    Outcome tryAdmit(Lifecycle lifecycle, RadioState radio) noexcept;

    /// Извлечь одну ожидающую заявку. false — ожидающих нет.
    /// Извлечённая заявка больше не входит в pending.
    bool takeOne() noexcept;

    /// Извлечь ВСЕ ожидающие заявки и учесть их как отброшенные.
    /// Не касается заявки, уже извлечённой через takeOne().
    std::uint8_t dropPending(DropReason reason) noexcept;

    /// Учёт результата уже извлечённой заявки (ровно один раз).
    void accountExecuted() noexcept;
    void accountFailed(DropReason reason) noexcept;

    std::uint8_t pending() const noexcept;

    std::atomic<std::uint32_t> accepted{0};
    std::atomic<std::uint32_t> rejected_not_running{0};
    std::atomic<std::uint32_t> rejected_unconfirmed{0};
    std::atomic<std::uint32_t> rejected_full{0};
    std::atomic<std::uint32_t> executed{0};
    std::atomic<std::uint32_t> failed_on_shutdown{0};
    std::atomic<std::uint32_t> failed_on_timeout{0};
    std::atomic<std::uint32_t> failed_on_error{0};
    std::atomic<std::uint32_t> dropped_on_shutdown{0};
    std::atomic<std::uint32_t> dropped_on_timeout{0};
    std::atomic<std::uint32_t> dropped_on_error{0};

private:
    void increment(std::atomic<std::uint32_t> &counter) noexcept;

    std::atomic<std::uint8_t> pending_{0};
};

}  // namespace wifi
