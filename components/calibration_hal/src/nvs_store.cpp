/**
 * @file nvs_store.cpp
 * @brief Чтение и запись записей калибровки через NVS.
 */

#include "calibration_hal/nvs_store.hpp"

#include <cstring>

#include "esp_log.h"
#include "nvs.h"

namespace calibration_hal {
namespace {

constexpr const char *kTag = "calib_nvs";
constexpr const char *kNamespace = "calib";

}  // namespace

const char *NvsStore::keyOf(motion::AxisId id, std::uint8_t slot)
{
    const bool azimuth = id == motion::AxisId::Azimuth;
    if (slot == 0) {
        return azimuth ? "aza" : "ala";
    }
    return azimuth ? "azb" : "alb";
}

esp_err_t NvsStore::init()
{
    nvs_handle_t handle = 0;
    const esp_err_t error = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (error != ESP_OK) {
        ESP_LOGE(kTag, "nvs_open failed: %s", esp_err_to_name(error));
        ready_ = false;
        return error;
    }
    nvs_close(handle);
    ready_ = true;
    return ESP_OK;
}

esp_err_t NvsStore::readSlot(const char *key, calibration::Record &out,
                             calibration::RecordResult &result)
{
    nvs_handle_t handle = 0;
    esp_err_t error = nvs_open(kNamespace, NVS_READONLY, &handle);
    if (error != ESP_OK) {
        result = calibration::RecordResult::InvalidLength;
        return error;
    }
    std::uint8_t buffer[calibration::kRecordBytes]{};
    std::size_t length = sizeof(buffer);
    error = nvs_get_blob(handle, key, buffer, &length);
    nvs_close(handle);
    if (error == ESP_ERR_NVS_NOT_FOUND) {
        result = calibration::RecordResult::InvalidLength;
        return error;
    }
    if (error != ESP_OK) {
        result = calibration::RecordResult::InvalidLength;
        return error;
    }
    if (length != calibration::kRecordBytes) {
        result = calibration::RecordResult::InvalidLength;
        return ESP_ERR_INVALID_SIZE;
    }
    result = calibration::decodeRecord(buffer, out);
    return ESP_OK;
}

NvsStore::Loaded NvsStore::load(motion::AxisId id)
{
    Loaded loaded{};
    if (!ready_) {
        loaded.error = ESP_ERR_INVALID_STATE;
        return loaded;
    }
    calibration::Record slot_a{};
    calibration::Record slot_b{};
    calibration::RecordResult result_a = calibration::RecordResult::InvalidLength;
    calibration::RecordResult result_b = calibration::RecordResult::InvalidLength;
    const esp_err_t error_a = readSlot(keyOf(id, 0), slot_a, result_a);
    const esp_err_t error_b = readSlot(keyOf(id, 1), slot_b, result_b);

    const calibration::Record *selected =
        calibration::selectRecord(&slot_a, result_a, &slot_b, result_b);
    if (selected != nullptr) {
        loaded.present = true;
        loaded.record = *selected;
        if (selected->sequence >= next_sequence_) {
            next_sequence_ = selected->sequence + 1;
        }
        return loaded;
    }
    // Отсутствие записи — не ошибка; ошибка чтения при наличии данных — да.
    if (error_a != ESP_ERR_NVS_NOT_FOUND && error_a != ESP_OK) {
        loaded.error = error_a;
    } else if (error_b != ESP_ERR_NVS_NOT_FOUND && error_b != ESP_OK) {
        loaded.error = error_b;
    }
    return loaded;
}

esp_err_t NvsStore::writeSlot(const char *key, const calibration::Record &record)
{
    std::uint8_t buffer[calibration::kRecordBytes]{};
    if (!calibration::encodeRecord(record, buffer)) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t handle = 0;
    const esp_err_t open_error = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (open_error != ESP_OK) {
        return open_error;
    }
    esp_err_t error = nvs_set_blob(handle, key, buffer, sizeof(buffer));
    if (error == ESP_OK) {
        error = nvs_commit(handle);
    }
    nvs_close(handle);
    return error;
}

esp_err_t NvsStore::save(motion::AxisId id, calibration::Record record)
{
    if (!ready_) {
        return ESP_ERR_INVALID_STATE;
    }
    calibration::Record slot_a{};
    calibration::Record slot_b{};
    calibration::RecordResult result_a = calibration::RecordResult::InvalidLength;
    calibration::RecordResult result_b = calibration::RecordResult::InvalidLength;
    (void)readSlot(keyOf(id, 0), slot_a, result_a);
    (void)readSlot(keyOf(id, 1), slot_b, result_b);

    const bool a_valid = result_a == calibration::RecordResult::Ok;
    const bool b_valid = result_b == calibration::RecordResult::Ok;
    std::uint8_t target = 0;
    if (a_valid && b_valid) {
        target = slot_a.sequence <= slot_b.sequence ? 0 : 1;
    } else if (a_valid) {
        target = 1;
    } else {
        target = 0;
    }

    record.sequence = next_sequence_;
    const esp_err_t error = writeSlot(keyOf(id, target), record);
    if (error == ESP_OK) {
        ++next_sequence_;
    }
    return error;
}

}  // namespace calibration_hal
