/**
 * @file retry_gate.hpp
 * @brief Отсчёт интервала между повторными попытками — без ESP-IDF.
 *
 * Метка ставится по завершению реально выполненной попытки, поэтому каждая
 * новая неудача заново отсчитывает полный интервал. Значение 0 — допустимое
 * время тика, поэтому отсутствие метки выражается отдельным флагом, а не
 * сравнением с нулём.
 *
 * Вычитание беззнаковых значений корректно работает и при переполнении
 * счётчика тиков.
 */

#pragma once

#include <cstdint>

namespace web {

class RetryGate {
public:
    /// Попытка выполнена и требует повтора: начинается новый интервал.
    void noteAttempt(std::uint32_t now_ticks) noexcept;

    /// Повтор больше не нужен.
    void clear() noexcept;

    /// Разрешён ли повтор в указанный момент.
    bool elapsed(std::uint32_t now_ticks,
                 std::uint32_t interval_ticks) const noexcept;

    bool armed() const noexcept { return armed_; }
    std::uint32_t attempts() const noexcept { return attempts_; }

private:
    std::uint32_t armed_at_ = 0;
    std::uint32_t attempts_ = 0;
    bool armed_ = false;
};

}  // namespace web
