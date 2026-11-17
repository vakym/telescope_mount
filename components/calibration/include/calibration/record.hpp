/**
 * @file record.hpp
 * @brief Постоянная запись калибровки оси: CRC, версия, двойной слот.
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "calibration/math.hpp"
#include "calibration/types.hpp"

namespace calibration {

constexpr std::uint32_t kRecordMagic = 0x43414C30U;  // "CAL0"
constexpr std::uint16_t kRecordSchemaVersion = 1;
constexpr std::size_t kRecordBytes = 36;

struct Record {
    std::uint32_t magic = kRecordMagic;
    std::uint16_t schema_version = kRecordSchemaVersion;
    std::uint16_t flags = 0;  // bit0: user_calibrated
    std::uint32_t sequence = 0;
    std::int64_t nanodegrees_per_half_step = 0;
    std::uint32_t sample_count = 0;
    std::uint32_t total_revolutions = 0;
    std::uint32_t spread_ppm = 0;

    bool userCalibrated() const noexcept { return (flags & 1U) != 0U; }
};

enum class RecordResult : std::uint8_t {
    Ok,
    InvalidMagic,
    InvalidVersion,
    InvalidLength,
    InvalidCrc,
    InvalidValue,
};

std::uint32_t crc32(const std::uint8_t *data, std::size_t length) noexcept;

/// Сериализация в фиксированный буфер kRecordBytes.
bool encodeRecord(const Record &record, std::uint8_t out[kRecordBytes]) noexcept;

/// Разбор и проверка записи. Не разыменовывает out при ошибке.
RecordResult decodeRecord(const std::uint8_t in[kRecordBytes], Record &out) noexcept;

/// Выбор валидной записи с максимальным sequence. nullptr — обе непригодны.
const Record *selectRecord(const Record *a, RecordResult result_a,
                           const Record *b, RecordResult result_b) noexcept;

Record makeNominalRecord(std::uint32_t nominal_half_steps_per_revolution,
                         std::uint32_t sequence) noexcept;

Record makeCalibratedRecord(const CombinedResult &result, std::uint32_t sequence) noexcept;

}  // namespace calibration
