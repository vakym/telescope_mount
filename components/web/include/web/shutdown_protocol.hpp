/**
 * @file shutdown_protocol.hpp
 * @brief Протокол завершения задачи-супервизора — чистая логика без ESP-IDF.
 *
 * Правила, которые устраняют уведомление удалённой задачи:
 *  - только первый запрос останова разрешает уведомление;
 *  - повторные вызовы (в том числе после таймаута) только ожидают
 *    подтверждение и никогда не уведомляют снова;
 *  - после подтверждения состояние терминально до reset(); задача удаляется
 *    владельцем, а не сама собой.
 */

#pragma once

#include <cstdint>

namespace web {

class ShutdownProtocol {
public:
    enum class WaitAction : std::uint8_t {
        NotifyThenWait,  ///< первый запрос: разбудить супервизор и ждать
        WaitOnly,        ///< повтор: только ждать, без уведомления
        Completed,       ///< завершение уже подтверждено
    };

    /// Запрос останова. Возвращает разрешённое действие вызывающего.
    WaitAction beginStop() noexcept;

    /// Ожидание не подтвердилось; следующий вызов снова только ждёт.
    void onWaitTimeout() noexcept;

    /// Подтверждение получено, задача удалена владельцем.
    void onCompleted() noexcept;

    /// Новый запуск: протокол снова в начальном состоянии.
    void reset() noexcept;

    bool stopped() const noexcept { return state_ == State::Completed; }
    std::uint32_t requests() const noexcept { return requests_; }
    std::uint32_t completions() const noexcept { return completions_; }
    std::uint32_t timeouts() const noexcept { return timeouts_; }

private:
    enum class State : std::uint8_t { Idle, Requested, Completed };

    State state_ = State::Idle;
    std::uint32_t requests_ = 0;
    std::uint32_t completions_ = 0;
    std::uint32_t timeouts_ = 0;
};

}  // namespace web
