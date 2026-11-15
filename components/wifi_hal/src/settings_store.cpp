/**
 * @file settings_store.cpp
 * @brief Kconfig-значения по умолчанию и переопределение из NVS.
 *
 * Приоритет: валидный ключ NVS переопределяет Kconfig, отсутствующий ключ
 * замещается Kconfig, невалидный игнорируется с предупреждением. Признак
 * непригодного значения Kconfig (`KconfigInvalid`) снимается только
 * успешным применением валидного значения NVS: невалидное сохранённое
 * значение его не скрывает.
 */

#include "wifi_hal/settings_store.hpp"

#include <atomic>
#include <cstddef>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

namespace wifi_hal {
namespace {

constexpr const char *kTag = "wifi_settings";
constexpr const char *kNamespace = "wifi";

constexpr const char *kKeySsid = "ssid";
constexpr const char *kKeyPassword = "pass";
constexpr const char *kKeyChannel = "chan";
constexpr const char *kKeyMaxStations = "maxsta";
constexpr const char *kKeyHidden = "hidden";
constexpr const char *kKeyStartOnBoot = "onboot";

/// Значения Kconfig не обрезаются. Строка, которая не помещается в буфер
/// или содержит непечатаемый символ, помечается невалидной и может быть
/// заменена только валидным переопределением из NVS.
wifi::AccessPointSettings makeDefaults(FieldStatus &ssid_status,
                                       FieldStatus &password_status) noexcept
{
    wifi::AccessPointSettings settings;
    if (!wifi::setSsid(settings.ap, CONFIG_WIFI_AP_SSID)) {
        ssid_status.issue = SettingIssue::KconfigInvalid;
    }
    if (!wifi::setPassword(settings.ap, CONFIG_WIFI_AP_PASSWORD)) {
        password_status.issue = SettingIssue::KconfigInvalid;
    }
    settings.ap.channel = static_cast<std::uint8_t>(CONFIG_WIFI_AP_CHANNEL);
    settings.ap.max_stations =
        static_cast<std::uint8_t>(CONFIG_WIFI_AP_MAX_STATIONS);
#ifdef CONFIG_WIFI_AP_HIDDEN
    settings.ap.hidden = true;
#else
    settings.ap.hidden = false;
#endif
#ifdef CONFIG_WIFI_AP_START_ON_BOOT
    settings.start_on_boot = true;
#else
    settings.start_on_boot = false;
#endif
    return settings;
}

/// Неверный тип и неверная длина — испорченное сохранённое значение, а не
/// ошибка чтения: поле замещается значением Kconfig.
bool isInvalidValueError(esp_err_t error) noexcept
{
    return error == ESP_ERR_NVS_TYPE_MISMATCH ||
           error == ESP_ERR_NVS_INVALID_LENGTH;
}

void markInvalid(FieldStatus &status, LoadResult &result) noexcept
{
    // Невалидное значение NVS не скрывает, что исходное значение Kconfig
    // непригодно: запуск в этом случае всё равно должен быть отвергнут.
    if (status.issue != SettingIssue::KconfigInvalid) {
        status.issue = SettingIssue::InvalidValue;
    }
    result.any_invalid = true;
}

void markReadError(FieldStatus &status, LoadResult &result,
                   esp_err_t error) noexcept
{
    status.issue = SettingIssue::ReadError;
    status.error = error;
    result.any_read_error = true;
    if (result.status == ESP_OK) {
        result.status = error;
    }
}

void markAllReadError(LoadResult &result, esp_err_t error) noexcept
{
    FieldStatus *fields[] = {&result.ssid,       &result.password,
                             &result.channel,    &result.max_stations,
                             &result.hidden,     &result.start_on_boot};
    for (FieldStatus *field : fields) {
        markReadError(*field, result, error);
    }
}

/**
 * @brief Читает строковое значение NVS.
 *
 * Возвращает три состояния: ключа нет, значение непригодно, значение
 * прочитано. Реальная ошибка чтения возвращается через `read_error`.
 */
wifi::NvsValue readStringValue(nvs_handle_t handle, const char *key,
                               char *buffer, std::size_t capacity,
                               esp_err_t &read_error) noexcept
{
    read_error = ESP_OK;
    std::size_t required = 0;
    esp_err_t err = nvs_get_str(handle, key, nullptr, &required);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return wifi::NvsValue::Absent;
    }
    if (isInvalidValueError(err)) {
        ESP_LOGW(kTag, "nvs key %s ignored: invalid stored type", key);
        return wifi::NvsValue::Invalid;
    }
    if (err != ESP_OK) {
        read_error = err;
        return wifi::NvsValue::Absent;
    }
    if (required == 0 || required > capacity) {
        // Содержимое значения не логируется: это может быть пароль.
        ESP_LOGW(kTag, "nvs key %s ignored: value does not fit", key);
        return wifi::NvsValue::Invalid;
    }
    err = nvs_get_str(handle, key, buffer, &required);
    if (isInvalidValueError(err)) {
        ESP_LOGW(kTag, "nvs key %s ignored: invalid stored value", key);
        return wifi::NvsValue::Invalid;
    }
    if (err != ESP_OK) {
        read_error = err;
        return wifi::NvsValue::Absent;
    }
    return wifi::NvsValue::Present;
}

void applyStringMerge(const wifi::FieldMerge &merge, wifi::NvsValue value,
                      FieldStatus &status, LoadResult &result) noexcept
{
    const wifi::FieldOutcome outcome =
        wifi::resolveField(!merge.kconfig_invalid, value, merge.applied);
    if (outcome.nvs_applied) {
        status.source = SettingSource::Nvs;
        status.issue = SettingIssue::None;
        return;
    }
    // Присутствующее, но отвергнутое значение NVS — тоже невалидность,
    // даже если значение Kconfig осталось валидным.
    if (outcome.nvs_rejected) {
        result.any_invalid = true;
    }
    switch (outcome.issue) {
        case wifi::FieldIssue::KconfigInvalid:
            status.issue = SettingIssue::KconfigInvalid;
            break;
        case wifi::FieldIssue::InvalidNvs:
            status.issue = SettingIssue::InvalidValue;
            break;
        case wifi::FieldIssue::None:
            break;
    }
}

bool readSsid(nvs_handle_t handle, wifi::AccessPointSettings &settings,
              FieldStatus &status, LoadResult &result) noexcept
{
    char buffer[wifi::kSsidCapacity]{};
    esp_err_t read_error = ESP_OK;
    const wifi::NvsValue value = readStringValue(
        handle, kKeySsid, buffer, wifi::kSsidCapacity, read_error);
    if (read_error != ESP_OK) {
        markReadError(status, result, read_error);
        return false;
    }

    const bool kconfig_valid = status.issue != SettingIssue::KconfigInvalid;
    const wifi::FieldMerge merge =
        wifi::mergeSsid(settings.ap, kconfig_valid, value, buffer);
    if (value == wifi::NvsValue::Present && !merge.applied) {
        ESP_LOGW(kTag, "nvs key %s ignored: invalid value", kKeySsid);
    }
    applyStringMerge(merge, value, status, result);
    return merge.applied;
}

bool readPassword(nvs_handle_t handle, wifi::AccessPointSettings &settings,
                  FieldStatus &status, LoadResult &result) noexcept
{
    char buffer[wifi::kPasswordCapacity]{};
    esp_err_t read_error = ESP_OK;
    const wifi::NvsValue value = readStringValue(
        handle, kKeyPassword, buffer, wifi::kPasswordCapacity, read_error);
    if (read_error != ESP_OK) {
        markReadError(status, result, read_error);
        return false;
    }

    const bool kconfig_valid = status.issue != SettingIssue::KconfigInvalid;
    const wifi::FieldMerge merge =
        wifi::mergePassword(settings.ap, kconfig_valid, value, buffer);
    if (value == wifi::NvsValue::Present && !merge.applied) {
        // Содержимое пароля не логируется.
        ESP_LOGW(kTag, "nvs key %s ignored: invalid value", kKeyPassword);
    }
    applyStringMerge(merge, value, status, result);
    return merge.applied;
}

bool readChannel(nvs_handle_t handle, wifi::AccessPointSettings &settings,
                 FieldStatus &status, LoadResult &result) noexcept
{
    std::uint8_t value = 0;
    const esp_err_t err = nvs_get_u8(handle, kKeyChannel, &value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    }
    if (isInvalidValueError(err)) {
        ESP_LOGW(kTag, "nvs key %s ignored: invalid stored type", kKeyChannel);
        markInvalid(status, result);
        return false;
    }
    if (err != ESP_OK) {
        markReadError(status, result, err);
        return false;
    }
    if (value < wifi::kChannelMin || value > wifi::kChannelMax) {
        ESP_LOGW(kTag, "nvs key %s ignored: value %u out of range",
                 kKeyChannel, static_cast<unsigned>(value));
        markInvalid(status, result);
        return false;
    }
    settings.ap.channel = value;
    status.source = SettingSource::Nvs;
    status.issue = SettingIssue::None;
    return true;
}

bool readMaxStations(nvs_handle_t handle, wifi::AccessPointSettings &settings,
                     FieldStatus &status, LoadResult &result) noexcept
{
    std::uint8_t value = 0;
    const esp_err_t err = nvs_get_u8(handle, kKeyMaxStations, &value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    }
    if (isInvalidValueError(err)) {
        ESP_LOGW(kTag, "nvs key %s ignored: invalid stored type",
                 kKeyMaxStations);
        markInvalid(status, result);
        return false;
    }
    if (err != ESP_OK) {
        markReadError(status, result, err);
        return false;
    }
    if (value < wifi::kMaxStationsMin || value > wifi::kMaxStationsMax) {
        ESP_LOGW(kTag, "nvs key %s ignored: value %u out of range",
                 kKeyMaxStations, static_cast<unsigned>(value));
        markInvalid(status, result);
        return false;
    }
    settings.ap.max_stations = value;
    status.source = SettingSource::Nvs;
    status.issue = SettingIssue::None;
    return true;
}

bool readFlag(nvs_handle_t handle, const char *key, bool &destination,
              FieldStatus &status, LoadResult &result) noexcept
{
    std::uint8_t value = 0;
    const esp_err_t err = nvs_get_u8(handle, key, &value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    }
    if (isInvalidValueError(err)) {
        ESP_LOGW(kTag, "nvs key %s ignored: invalid stored type", key);
        markInvalid(status, result);
        return false;
    }
    if (err != ESP_OK) {
        markReadError(status, result, err);
        return false;
    }
    // Принимается строго 0 или 1, а не любой ненулевой байт.
    if (value > 1) {
        ESP_LOGW(kTag, "nvs key %s ignored: value %u is not 0 or 1", key,
                 static_cast<unsigned>(value));
        markInvalid(status, result);
        return false;
    }
    destination = value == 1;
    status.source = SettingSource::Nvs;
    status.issue = SettingIssue::None;
    return true;
}

bool readHidden(nvs_handle_t handle, wifi::AccessPointSettings &settings,
                FieldStatus &status, LoadResult &result) noexcept
{
    return readFlag(handle, kKeyHidden, settings.ap.hidden, status, result);
}

bool readStartOnBoot(nvs_handle_t handle,
                     wifi::AccessPointSettings &settings,
                     FieldStatus &status, LoadResult &result) noexcept
{
    return readFlag(handle, kKeyStartOnBoot, settings.start_on_boot, status,
                    result);
}

}  // namespace

esp_err_t SettingsStore::initNvs()
{
    static std::atomic<bool> initialized{false};
    if (initialized.load(std::memory_order_acquire)) {
        return ESP_OK;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(kTag, "nvs init needs erase (%s): whole partition is erased",
                 esp_err_to_name(err));
        err = nvs_flash_erase();
        if (err != ESP_OK) {
            ESP_LOGE(kTag, "nvs erase failed: %s", esp_err_to_name(err));
            return err;
        }
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "nvs init failed: %s", esp_err_to_name(err));
        return err;
    }
    initialized.store(true, std::memory_order_release);
    return ESP_OK;
}

LoadResult SettingsStore::load() const
{
    LoadResult result;
    result.settings = makeDefaults(result.ssid, result.password);

#ifdef CONFIG_WIFI_SETTINGS_FROM_NVS
    nvs_handle_t handle = 0;
    const esp_err_t open = nvs_open(kNamespace, NVS_READONLY, &handle);
    if (open == ESP_ERR_NVS_NOT_FOUND) {
        return result;  // namespace отсутствует: только Kconfig
    }
    if (open != ESP_OK) {
        markAllReadError(result, open);
        return result;
    }

    (void)readSsid(handle, result.settings, result.ssid, result);
    (void)readPassword(handle, result.settings, result.password, result);
    (void)readChannel(handle, result.settings, result.channel, result);
    (void)readMaxStations(handle, result.settings, result.max_stations, result);
    (void)readHidden(handle, result.settings, result.hidden, result);
    (void)readStartOnBoot(handle, result.settings, result.start_on_boot, result);

    nvs_close(handle);
#endif

    return result;
}

esp_err_t SettingsStore::save(const wifi::AccessPointSettings &settings)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "nvs open for write failed: %s", esp_err_to_name(err));
        return err;
    }

    if (err == ESP_OK) {
        err = nvs_set_str(handle, kKeySsid, settings.ap.ssid);
    }
    if (err == ESP_OK) {
        err = nvs_set_str(handle, kKeyPassword, settings.ap.password);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, kKeyChannel, settings.ap.channel);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, kKeyMaxStations, settings.ap.max_stations);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, kKeyHidden, settings.ap.hidden ? 1 : 0);
    }
    if (err == ESP_OK) {
        err = nvs_set_u8(handle, kKeyStartOnBoot,
                         settings.start_on_boot ? 1 : 0);
    }
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(kTag, "nvs save failed: %s", esp_err_to_name(err));
    }
    return err;
}

}  // namespace wifi_hal
