#include "calibration/record.hpp"

#include <limits>

namespace calibration {
namespace {

void put16(std::uint8_t *out, std::uint16_t value) noexcept
{
    out[0] = static_cast<std::uint8_t>(value & 0xFFU);
    out[1] = static_cast<std::uint8_t>((value >> 8) & 0xFFU);
}

void put32(std::uint8_t *out, std::uint32_t value) noexcept
{
    for (int i = 0; i < 4; ++i) {
        out[i] = static_cast<std::uint8_t>((value >> (8 * i)) & 0xFFU);
    }
}

void put64(std::uint8_t *out, std::uint64_t value) noexcept
{
    for (int i = 0; i < 8; ++i) {
        out[i] = static_cast<std::uint8_t>((value >> (8 * i)) & 0xFFU);
    }
}

std::uint16_t get16(const std::uint8_t *in) noexcept
{
    return static_cast<std::uint16_t>(in[0]) |
           static_cast<std::uint16_t>(static_cast<std::uint16_t>(in[1]) << 8);
}

std::uint32_t get32(const std::uint8_t *in) noexcept
{
    std::uint32_t value = 0;
    for (int i = 0; i < 4; ++i) {
        value |= static_cast<std::uint32_t>(in[i]) << (8 * i);
    }
    return value;
}

std::uint64_t get64(const std::uint8_t *in) noexcept
{
    std::uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value |= static_cast<std::uint64_t>(in[i]) << (8 * i);
    }
    return value;
}

bool validRecordValue(const Record &record) noexcept
{
    if (record.nanodegrees_per_half_step <= 0) {
        return false;
    }
    if (record.sample_count > 1000U || record.total_revolutions > 100000U) {
        return false;
    }
    if ((record.flags & 1U) == 0U &&
        (record.sample_count != 0U || record.total_revolutions != 0U)) {
        return false;
    }
    return true;
}

}  // namespace

std::uint32_t crc32(const std::uint8_t *data, std::size_t length) noexcept
{
    std::uint32_t crc = 0xFFFFFFFFU;
    if (data == nullptr) {
        return 0U;
    }
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

bool encodeRecord(const Record &record, std::uint8_t out[kRecordBytes]) noexcept
{
    if (out == nullptr || !validRecordValue(record)) {
        return false;
    }
    put32(out + 0, record.magic);
    put16(out + 4, record.schema_version);
    put16(out + 6, record.flags);
    put32(out + 8, record.sequence);
    put64(out + 12, static_cast<std::uint64_t>(record.nanodegrees_per_half_step));
    put32(out + 20, record.sample_count);
    put32(out + 24, record.total_revolutions);
    put32(out + 28, record.spread_ppm);
    put32(out + 32, crc32(out, 32));
    return true;
}

RecordResult decodeRecord(const std::uint8_t in[kRecordBytes], Record &out) noexcept
{
    if (in == nullptr) {
        return RecordResult::InvalidLength;
    }
    if (get32(in + 0) != kRecordMagic) {
        return RecordResult::InvalidMagic;
    }
    if (get16(in + 4) != kRecordSchemaVersion) {
        return RecordResult::InvalidVersion;
    }
    if (get32(in + 32) != crc32(in, 32)) {
        return RecordResult::InvalidCrc;
    }
    Record record{};
    record.magic = kRecordMagic;
    record.schema_version = kRecordSchemaVersion;
    record.flags = get16(in + 6);
    record.sequence = get32(in + 8);
    record.nanodegrees_per_half_step =
        static_cast<std::int64_t>(get64(in + 12));
    record.sample_count = get32(in + 20);
    record.total_revolutions = get32(in + 24);
    record.spread_ppm = get32(in + 28);
    if (!validRecordValue(record)) {
        return RecordResult::InvalidValue;
    }
    out = record;
    return RecordResult::Ok;
}

const Record *selectRecord(const Record *a, RecordResult result_a,
                           const Record *b, RecordResult result_b) noexcept
{
    const bool a_ok = a != nullptr && result_a == RecordResult::Ok;
    const bool b_ok = b != nullptr && result_b == RecordResult::Ok;
    if (!a_ok && !b_ok) {
        return nullptr;
    }
    if (!a_ok) {
        return b;
    }
    if (!b_ok) {
        return a;
    }
    return b->sequence > a->sequence ? b : a;
}

Record makeNominalRecord(std::uint32_t nominal_half_steps_per_revolution,
                         std::uint32_t sequence) noexcept
{
    Record record{};
    record.sequence = sequence;
    record.flags = 0;
    record.nanodegrees_per_half_step = toNanodegrees(
        nominalDegreesPerHalfStep(nominal_half_steps_per_revolution));
    record.sample_count = 0;
    record.total_revolutions = 0;
    record.spread_ppm = 0;
    return record;
}

Record makeCalibratedRecord(const CombinedResult &result, std::uint32_t sequence) noexcept
{
    Record record{};
    record.sequence = sequence;
    record.flags = 1;
    record.nanodegrees_per_half_step =
        toNanodegrees(result.degrees_per_half_step);
    record.sample_count = result.sample_count;
    record.total_revolutions = result.total_revolutions;
    record.spread_ppm = result.spread_ppm;
    return record;
}

}  // namespace calibration
