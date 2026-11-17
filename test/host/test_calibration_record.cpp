#include <cstdint>

#include "calibration/record.hpp"
#include "check.hpp"

using namespace calibration;

int main()
{
    const Record nominal = makeNominalRecord(4096, 7);
    CHECK(nominal.nanodegrees_per_half_step == 87890625);
    CHECK(!nominal.userCalibrated());

    std::uint8_t buffer[kRecordBytes]{};
    CHECK(encodeRecord(nominal, buffer));

    Record decoded{};
    CHECK(decodeRecord(buffer, decoded) == RecordResult::Ok);
    CHECK(decoded.sequence == 7);
    CHECK(decoded.nanodegrees_per_half_step == 87890625);
    CHECK(!decoded.userCalibrated());

    // CRC corruption.
    std::uint8_t broken[kRecordBytes]{};
    for (std::size_t i = 0; i < kRecordBytes; ++i) {
        broken[i] = buffer[i];
    }
    broken[kRecordBytes - 1] ^= 0x5AU;
    Record rejected{};
    CHECK(decodeRecord(broken, rejected) == RecordResult::InvalidCrc);

    // Unknown schema version.
    std::uint8_t versioned[kRecordBytes]{};
    for (std::size_t i = 0; i < kRecordBytes; ++i) {
        versioned[i] = buffer[i];
    }
    versioned[4] = 9;
    CHECK(decodeRecord(versioned, rejected) == RecordResult::InvalidVersion);

    // User-calibrated record and slot selection.
    CombinedResult result{};
    result.ok = true;
    result.degrees_per_half_step = 0.087821;
    result.half_steps_per_revolution = 4099.333;
    result.sample_count = 3;
    result.total_revolutions = 9;
    result.spread_ppm = 500;
    const Record user = makeCalibratedRecord(result, 8);
    CHECK(user.userCalibrated());
    CHECK(user.sample_count == 3);
    std::uint8_t user_bytes[kRecordBytes]{};
    CHECK(encodeRecord(user, user_bytes));
    Record user_decoded{};
    CHECK(decodeRecord(user_bytes, user_decoded) == RecordResult::Ok);

    Record second = nominal;
    second.sequence = 6;
    const Record *chosen = selectRecord(&user, RecordResult::Ok, &second, RecordResult::Ok);
    CHECK(chosen == &user);
    CHECK(selectRecord(nullptr, RecordResult::InvalidCrc, &second, RecordResult::Ok) == &second);
    CHECK(selectRecord(nullptr, RecordResult::InvalidCrc, nullptr, RecordResult::InvalidCrc) ==
          nullptr);

    return TEST_RESULT();
}
