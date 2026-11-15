/**
 * @file executor_io.hpp
 * @brief Интерфейсы, через которые AxisExecutor взаимодействует с железом.
 *
 * Реализации для ESP-IDF находятся в motor_hal, тестовые — в test/host.
 * Благодаря этому событийный цикл исполняется одним и тем же кодом на хосте
 * и на микроконтроллере.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "motion/axis_command.hpp"
#include "motion/types.hpp"

namespace motion {

/// Источник времени и одноразовых alarm на абсолютные дедлайны.
class AlarmSink {
public:
    virtual ~AlarmSink() = default;
    virtual bool armAt(std::uint64_t deadline_us) = 0;
    virtual void disarm() = 0;
    virtual std::uint64_t nowUs() const = 0;
};

/// Управление фазами ULN2003.
class CoilSink {
public:
    virtual ~CoilSink() = default;
    virtual void advance(Direction direction) = 0;
    virtual void release() = 0;
    virtual void energize() = 0;
    virtual bool isEnergized() const = 0;
};

/// Очередь команд оси и канал приоритетного останова.
class CommandSource {
public:
    virtual ~CommandSource() = default;
    virtual std::size_t take(AxisCommand *out, std::size_t max_count) = 0;
    virtual bool empty() const = 0;
    virtual void clear() = 0;
    /**
     * @brief Атомарно забирает идентификатор ожидающего emergency-запроса.
     *
     * 0 означает «отслеживаемого запроса нет». Забирать нужно первым
     * действием обработки останова, иначе подтверждается запрос, для
     * которого очередь уже была очищена ранее.
     */
    virtual std::uint64_t takeEmergencyId() { return 0; }
};

}  // namespace motion
