/**
 * @file ap_lifecycle.hpp
 * @brief Координатор операций точки доступа — чистая логика без ESP-IDF.
 *
 * Разделены два независимых понятия:
 *  - `Lifecycle` — жизненный цикл службы;
 *  - `RadioState` — подтверждённое событиями состояние радио.
 *
 * Таймаут ожидания события не доказывает, что операция не выполнилась:
 * он переводит радио в `Unknown`. Подтверждением операции считается только
 * событие того же направления либо явный результат вызова драйвера.
 * Время координатор не измеряет: таймаут приходит извне вызовом
 * `onOperationTimeout()`.
 */

#pragma once

#include <cstdint>

namespace wifi {

enum class Lifecycle : std::uint8_t { Stopped, Starting, Running, Stopping };
enum class RadioState : std::uint8_t { Unknown, Running, Stopped };
enum class Operation : std::uint8_t { None, Starting, Stopping, Resyncing };
enum class Action : std::uint8_t { None, StartRadio, StopRadio, ResyncStop };

class ApCoordinator {
public:
    void beginStartup() noexcept;
    void startupSucceeded(bool ap_confirmed_running) noexcept;
    void startupFailed() noexcept;

    /// Операция первоначального запуска AP. Взводит `Starting`, чтобы
    /// служба ждала AP_START, а не считала состояние несовпавшим.
    Action beginInitialStart() noexcept;

    /// Действие для уже извлечённой заявки. None — состояние не позволяет
    /// начать операцию; служба считает это внутренней ошибкой.
    Action beginToggleOperation() noexcept;

    /// Одна попытка ресинхронизации по явному поводу.
    Action beginResync() noexcept;

    void onApStarted() noexcept;
    void onApStopped() noexcept;
    /// Нет подтверждения: состояние становится Unknown.
    void onOperationTimeout() noexcept;
    /// Немедленная ошибка вызова драйвера: состояние становится Unknown.
    void onOperationFailed() noexcept;
    /// Документированный безопасный результат «уже остановлено».
    void onAlreadyStopped() noexcept;
    void onStaConnected() noexcept;
    void onStaDisconnected() noexcept;

    bool beginShutdown() noexcept;
    void finishShutdown() noexcept;

    Lifecycle lifecycle() const noexcept { return lifecycle_; }
    RadioState radioState() const noexcept { return radio_; }
    Operation operation() const noexcept { return operation_; }
    std::uint8_t stations() const noexcept { return stations_; }

    std::uint32_t event_timeouts = 0;
    std::uint32_t operation_errors = 0;
    std::uint32_t resyncs = 0;
    std::uint32_t starts = 0;
    std::uint32_t stops = 0;

private:
    Lifecycle lifecycle_ = Lifecycle::Stopped;
    RadioState radio_ = RadioState::Unknown;
    Operation operation_ = Operation::None;
    std::uint8_t stations_ = 0;
};

}  // namespace wifi
