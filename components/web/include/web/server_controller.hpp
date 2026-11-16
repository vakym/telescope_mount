/**
 * @file server_controller.hpp
 * @brief Обёртка над реальным сервером: правила владения и финализации.
 *
 * Чистая логика без ESP-IDF. HAL-слой реализует `ServerPort` (запуск,
 * останов, наличие сервера), а контроллер применяет те же правила, что и
 * автомат `ServerLifecycle`:
 *  - `Stopped` достигается только при подтверждённом останове;
 *  - при неподтверждённом останове владение сохраняется, а не «забывается»;
 *  - терминальный неподтверждённый останов запрещает уничтожение объекта:
 *    вызывающий обязан сохранить владельца.
 *
 * Именно поэтому финализация задачи-супервизора выполняется через
 * `shutdownOnce()`, а не отдельным вызовом остановки.
 */

#pragma once

#include <cstdint>

#include "web/server_lifecycle.hpp"

namespace web {

/// Итог запуска сервера и регистрации маршрутов.
enum class StartOutcome : std::uint8_t {
    Started,                     ///< сервер работает, все маршруты есть
    Failed,                      ///< сервер не создан или откат подтверждён
    FailedServerLiveRetryable,   ///< откат не подтверждён, повтор разрешён
    FailedServerLiveTerminal,    ///< откат не подтверждён, повтор невозможен
};

/// Итог одной попытки останова.
enum class StopOutcome : std::uint8_t {
    Confirmed,              ///< останов подтверждён, handle освобождён
    NotConfirmedRetryable,  ///< сервер жив по контракту, повтор допустим
    NotConfirmedTerminal,   ///< код не доказывает освобождение, повтора нет
};

/// Действия с реальным сервером. Реализуется HAL-слоем.
class ServerPort {
public:
    virtual ~ServerPort() = default;

    virtual StartOutcome start() noexcept = 0;
    virtual StopOutcome stop() noexcept = 0;

    /// Сервер (handle) существует и требует останова.
    virtual bool hasServer() const noexcept = 0;
};

/// Итог одного шага финализации.
enum class ShutdownOutcome : std::uint8_t {
    Clean,     ///< сервера нет или останов подтверждён
    Retry,     ///< останов не подтверждён, нужен повтор
    Terminal,  ///< владение сохраняется, объект уничтожать нельзя
};

class ServerController {
public:
    explicit ServerController(ServerPort &port) noexcept : port_(port) {}

    /// Один тик: не более одного действия с сервером.
    ServerAction tick(bool ap_running, bool ap_stopping,
                      bool retry_elapsed) noexcept;

    /// Один шаг финализации; вызывающий повторяет его до Clean или Terminal.
    ShutdownOutcome shutdownOnce() noexcept;

    ServerState state() const noexcept { return lifecycle_.state(); }
    const ServerLifecycle &lifecycle() const noexcept { return lifecycle_; }

    /// Нужен повтор действия; HAL вычисляет, истёк ли интервал.
    bool retryPending() const noexcept { return retry_pending_; }

private:
    void applyStop(StopOutcome outcome) noexcept;

    ServerPort &port_;
    ServerLifecycle lifecycle_{};
    bool retry_pending_ = false;
};

}  // namespace web
