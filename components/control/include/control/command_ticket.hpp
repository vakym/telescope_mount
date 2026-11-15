/**
 * @file command_ticket.hpp
 * @brief Тикет команды и результат её приёма к исполнению.
 *
 * Идентификатор назначает нижний слой (AxisMotor), а не верхний: иначе
 * перезапуск службы дал бы коллизию с прошлым подтверждением.
 */

#pragma once

#include <cstdint>

namespace control {

/// Тикет конкретной команды: идентификатор и эпоха запуска оси.
struct CommandTicket {
    std::uint64_t id = 0;
    std::uint32_t epoch = 0;

    bool valid() const noexcept { return id != 0U && epoch != 0U; }
};

/**
 * @brief Результат приёма команды.
 *
 * accepted: для FIFO-команды — фактическое присутствие в очереди;
 * для Immediate — ненулевой id опубликован в emergency-слоте.
 * notified: удалось ли разбудить задачу оси (только FIFO-путь). Команда с
 * accepted == true и notified == false не повторяется: она уже принята.
 */
struct Delivery {
    bool accepted = false;
    bool notified = false;
    CommandTicket ticket{};
};

}  // namespace control
