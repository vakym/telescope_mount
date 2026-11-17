/**
 * @file calibration_json.hpp
 * @brief Контракт `GET /api/calibration`: ограниченная сериализация.
 *
 * Компонент web — чистый C++17. Он не знает про ESP-IDF и NVS: получает
 * готовое представление и пишет документ фиксированного размера.
 * `saved` (булев признак пользовательской калибровки) и `persistence`
 * (состояние хранения) — разные поля.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "calibration/math.hpp"
#include "calibration/types.hpp"

namespace web {

/// Верхняя граница документа статуса калибровки.
constexpr std::size_t kMaxCalibrationJsonBytes = 2048;

struct CalibrationView {
    calibration::SessionState state = calibration::SessionState::Inactive;
    calibration::SessionError error = calibration::SessionError::None;
    calibration::InvalidReason invalid_reason = calibration::InvalidReason::Ok;
    std::uint32_t session_id = 0;
    std::uint32_t revision = 0;
    std::uint32_t motion_generation = 0;
    motion::AxisId axis = motion::AxisId::Azimuth;
    motion::Direction direction = motion::Direction::Forward;
    std::uint32_t revolutions = 0;
    std::uint32_t repeats = 0;
    std::uint32_t measurement_count = 0;
    calibration::CombinedResult result{};
    bool mark_recheck_required = false;
    bool hold_active = false;
    bool input_ready = false;
    bool flash_window_active = false;
    std::uint32_t lease_remaining_ms = 0;
    calibration::AxisScale scale[2]{};
    std::uint32_t nominal_half_steps_per_revolution[2] = {0, 0};
};

enum class CalibrationJsonResult : std::uint8_t { Ok, Overflow };

CalibrationJsonResult calibrationStatusJson(const CalibrationView &view, char *out,
                                            std::size_t capacity,
                                            std::size_t *written) noexcept;

}  // namespace web
