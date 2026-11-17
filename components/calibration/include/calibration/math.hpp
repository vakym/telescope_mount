/**
 * @file math.hpp
 * @brief Расчёт масштаба по подтверждённым счётчикам полушагов.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "calibration/types.hpp"

namespace calibration {

/// Безопасная беззнаковая разность двух int64 без переполнения.
std::uint64_t safeDelta(std::int64_t a, std::int64_t b) noexcept;

/// Создаёт измерение из двух подтверждённых счётчиков.
Measurement makeMeasurement(std::uint32_t index, std::uint32_t revolutions,
                            std::int64_t start_position,
                            std::int64_t end_position,
                            motion::Direction expected_direction,
                            std::uint64_t start_ticket,
                            std::uint64_t stop_ticket,
                            std::uint32_t epoch,
                            const CalibrationParams &params) noexcept;

struct CombinedResult {
    bool ok = false;
    InvalidReason reason = InvalidReason::InvalidResult;
    double degrees_per_half_step = 0.0;
    double half_steps_per_revolution = 0.0;
    std::uint32_t sample_count = 0;
    std::uint32_t total_revolutions = 0;
    std::uint32_t spread_ppm = 0;
    std::uint32_t max_deviation_ppm = 0;
    Repeatability repeatability = Repeatability::Insufficient;
};

/**
 * Итог: k = 360 * sum(M_i) / sum(delta_i).
 * Входят только valid-измерения. Простое среднее k_i не используется.
 */
CombinedResult combine(const Measurement *measurements, std::size_t count,
                       const CalibrationParams &params) noexcept;

/// Номинальный коэффициент из числа полушагов на оборот.
double nominalDegreesPerHalfStep(std::uint32_t half_steps_per_revolution) noexcept;

/// Знаковая исполнительная уставка из направления и положительного модуля.
/// Направление — единственный источник знака: клиент передаёт модуль.
float signedRate(motion::Direction direction, float magnitude) noexcept;

/// Действует ли ненулевое браузерное разрешение: остаток строго больше
/// минимального срока уставки. Единственное определение границы.
bool movePermissionValid(bool move_active, Micros now, Micros until,
                         Micros min_lease) noexcept;

/// Перевод коэффициента в наноградусы на полушаг с округлением.
std::int64_t toNanodegrees(double degrees_per_half_step) noexcept;

}  // namespace calibration
