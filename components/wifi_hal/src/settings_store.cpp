/**
 * @file settings_store.cpp
 * @brief Kconfig-значения по умолчанию и переопределение из NVS.
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

void copyBounded(char *destination, std::size_t capacity,
                 const char *source) noexcept
{
    std::size_t i = 0;
    while (i + 1 < capacity && source[i] != '\0') {
        destination[i] = source[i];
        ++i;
    }
    destination[i] = '\0';
}

wifi::AccessPointSettings makeDefaults() noexcept
{
    wifi::AccessPointSettings settings;
    copyBounded(settings.ap.ssid, wifi::kSsidCapacity, CONFIG_WIFI_AP_SSID);
    copyBounded(settings.ap.password, wifi::kPasswordCapacity,
                CONFIG_WIFI_AP_PASSWORD);
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

/// Опорные настройки с заведомо валидными полями: позволяют проверить
/// одно значение NVS независимо от остальных.
wifi::AccessPointSettings makeProbe() noexcept
{
    wifi::AccessPointSettings probe;
    (void)wifi::setSsid(probe.ap, "probe");
    (void)wifi::setPassword(probe.ap, "12345678");
    probe.ap.channel = wifi::kChannelMin;
    probe.ap.max_stations = wifi::kMaxStationsMin;
    return probe;
}

bool acceptableSsid(const char *value) noexcept
{
    wifi::AccessPointSettings probe = makeProbe();
    if (!wifi::setSsid(probe.ap, value)) {
        return false;
    }
    return wifi::validate(probe) == wifi::ConfigError::None;
}

bool acceptablePassword(const char *value) noexcept
{
    wifi::AccessPointSettings probe = makeProbe();
    if (!wifi::setPassword(probe.ap, value)) {
        return false;
    }
    return wifi::validate(probe) == wifi::ConfigError::None;
}

void markInvalid(FieldStatus &status, LoadResult &result) noexcept
{
    status.issue = SettingIssue::InvalidValue;
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

bool readSsid(nvs_handle_t handle, wifi::AccessPointSettings &settings,
              FieldStatus &status, LoadResult &result) noexcept
{
    std::size_t required = 0;
    esp_err_t err = nvs_get_str(handle, kKeySsid, nullptr, &required);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    }
    if (err != ESP_OK) {
        markReadError(status, result, err);
        return false;
    }
    if (required == 0 || required > wifi::kSsidCapacity) {
        ESP_LOGW(kTag, "nvs key %s ignored: value does not fit", kKeySsid);
        markInvalid(status, result);
        return false;
    }
    char buffer[wifi::kSsidCapacity]{};
    err = nvs_get_str(handle, kKeySsid, buffer, &required);
    if (err != ESP_OK) {
        markReadError(status, result, err);
        return false;
    }
    if (!acceptableSsid(buffer)) {
        ESP_LOGW(kTag, "nvs key %s ignored: invalid value", kKeySsid);
        markInvalid(status, result);
        return false;
    }
    (void)wifi::setSsid(settings.ap, buffer);
    status.source = SettingSource::Nvs;
    return true;
}

bool readPassword(nvs_handle_t handle, wifi::AccessPointSettings &settings,
                  FieldStatus &status, LoadResult &result) noexcept
{
    std::size_t required = 0;
    esp_err_t err = nvs_get_str(handle, kKeyPassword, nullptr, &required);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return false;
    }
    if (err != ESP_OK) {
        markReadError(status, result, err);
        return false;
    }
    if (required == 0 || required > wifi::kPasswordCapacity) {
        // Значение не логируется: это пароль.
        ESP_LOGW(kTag, "nvs key %s ignored: value does not fit", kKeyPassword);
        markInvalid(status, result);
        return false;
    }
    char buffer[wifi::kPasswordCapacity]{};
    err = nvs_get_str(handle, kKeyPassword, buffer, &required);
    if (err != ESP_OK) {
        markReadError(status, result, err);
        return false;
    }
    if (!acceptablePassword(buffer)) {
        ESP_LOGW(kTag, "nvs key %s ignored: invalid value", kKeyPassword);
        markInvalid(status, result);
        return false;
    }
    (void)wifi::setPassword(settings.ap, buffer);
    status.source = SettingSource::Nvs;
    return true;
}

bool readChannel(nvs_handle_t handle, wifi::AccessPointSettings &settings,
                 FieldStatus &status, LoadResult &result) noexcept
{
    std::uint8_t value = 0;
    const esp_err_t err = nvs_get_u8(handle, kKeyChannel, &value);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
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
    result.settings = makeDefaults();

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
