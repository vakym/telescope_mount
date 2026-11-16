/**
 * @file server_lifecycle.hpp
 * @brief Автомат запуска и останова HTTP-сервера — чистая логика без ESP-IDF.
 *
 * Владелец ресурсов один: HAL-слой вызывает update() на каждом тике и
 * выполняет выданное действие. Время автомат не измеряет — признак
 * «повтор разрешён» приходит снаружи, как в wifi::ApCoordinator.
 *
 * Ключевое правило: состояние `Stopped` и «забывание» handle допустимы
 * только после **подтверждённого** завершения. Ненулевой результат
 * `httpd_stop()` сам по себе этого не доказывает: в ESP-IDF 6.1 `ESP_FAIL`
 * означает, что сервер продолжает работать, а `httpd_stop()` не имеет
 * таймаута на пути успеха.
 */

#pragma once

#include <cstdint>

namespace web {

enum class ServerState : std::uint8_t {
    Stopped,      ///< сервера нет, handle забыт
    Starting,     ///< выдан Start, ждём результат
    Running,      ///< сервер работает
    Stopping,     ///< выдан Stop, ждём подтверждения
    StopFailed,   ///< завершение не подтверждено; Start запрещён
    StartFailed,  ///< запуск не удался; повтор по разрешению снаружи
};

enum class ServerAction : std::uint8_t { None, Start, Stop };

class ServerLifecycle {
public:
    /**
     * @brief Решение на текущем тике.
     *
     * @param ap_running радио точки доступа подтверждено работающим
     * @param ap_stopping радио выключается или выключено
     * @param retry_elapsed интервал повторной попытки истёк
     */
    ServerAction update(bool ap_running, bool ap_stopping,
                        bool retry_elapsed) noexcept;

    /// Сервер создан и все маршруты зарегистрированы.
    void startSucceeded() noexcept;

    /// Запуск или регистрация маршрутов не удались; частичный старт откатан.
    void startFailed() noexcept;

    /**
     * @brief Старт не удался, и откат останова не подтверждён.
     *
     * Сервер мог остаться живым, поэтому `StartFailed` недопустим: он снова
     * разрешил бы запуск. При `retry_allowed` автомат переходит в `Stopping`
     * и повторит останов; иначе — терминальный `StopFailed`.
     */
    void startFailedWithLiveServer(bool retry_allowed) noexcept;

    /// Подтверждённое завершение: ресурсы освобождены, handle недействителен.
    void stopSucceeded() noexcept;

    /**
     * @brief Завершение не подтверждено.
     *
     * @param retry_allowed контракт библиотеки допускает повторный вызов
     *        останова на том же handle
     */
    void stopUnconfirmed(bool retry_allowed) noexcept;

    ServerState state() const noexcept { return state_; }

    std::uint32_t starts = 0;
    std::uint32_t starts_confirmed = 0;
    std::uint32_t start_failures = 0;
    std::uint32_t stops = 0;
    std::uint32_t stops_confirmed = 0;
    std::uint32_t stop_unconfirmed = 0;
    std::uint32_t stop_terminal = 0;

private:
    ServerState state_ = ServerState::Stopped;
    /// Повтор останова разрешён только после неподтверждённого результата.
    bool stop_retry_pending_ = false;
};

}  // namespace web
