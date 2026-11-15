/**
 * @file ap_config.cpp
 * @brief Валидация параметров точки доступа и безопасная запись строк.
 */

#include "wifi/ap_config.hpp"

namespace wifi {
namespace {

bool isPrintable(char value) noexcept
{
    const auto ch = static_cast<unsigned char>(value);
    return ch >= 0x20U && ch <= 0x7EU;
}

/// Длина строки в пределах capacity. Возвращает capacity, если терминатора
/// внутри массива нет.
std::size_t boundedLength(const char *buffer, std::size_t capacity) noexcept
{
    std::size_t length = 0;
    while (length < capacity && buffer[length] != '\0') {
        ++length;
    }
    return length;
}

bool allPrintable(const char *buffer, std::size_t length) noexcept
{
    for (std::size_t i = 0; i < length; ++i) {
        if (!isPrintable(buffer[i])) {
            return false;
        }
    }
    return true;
}

/// Копирование с терминатором. Вызывается только после проверки длины.
void copyString(char *destination, const char *source,
                std::size_t length) noexcept
{
    for (std::size_t i = 0; i < length; ++i) {
        destination[i] = source[i];
    }
    destination[length] = '\0';
}

ConfigError validateSsid(const char *ssid) noexcept
{
    const std::size_t length = boundedLength(ssid, kSsidCapacity);
    if (length == kSsidCapacity) {
        return ConfigError::SsidNotTerminated;
    }
    if (length == 0) {
        return ConfigError::EmptySsid;
    }
    if (length > kSsidMaxLength) {
        return ConfigError::SsidTooLong;
    }
    if (!allPrintable(ssid, length)) {
        return ConfigError::SsidNotPrintable;
    }
    return ConfigError::None;
}

ConfigError validatePassword(const char *password) noexcept
{
    const std::size_t length = boundedLength(password, kPasswordCapacity);
    if (length == kPasswordCapacity) {
        return ConfigError::PasswordNotTerminated;
    }
    if (length > kPasswordMaxLength) {
        return ConfigError::PasswordTooLong;
    }
    // Пустой пароль — открытая сеть; WPA2-PSK требует не меньше 8 символов.
    if (length > 0 && length < 8) {
        return ConfigError::PasswordTooShort;
    }
    if (!allPrintable(password, length)) {
        return ConfigError::PasswordNotPrintable;
    }
    return ConfigError::None;
}

}  // namespace

ConfigError validate(const AccessPointSettings &settings) noexcept
{
    const ConfigError ssid_error = validateSsid(settings.ap.ssid);
    if (ssid_error != ConfigError::None) {
        return ssid_error;
    }
    const ConfigError password_error = validatePassword(settings.ap.password);
    if (password_error != ConfigError::None) {
        return password_error;
    }
    if (settings.ap.channel < kChannelMin ||
        settings.ap.channel > kChannelMax) {
        return ConfigError::InvalidChannel;
    }
    if (settings.ap.max_stations < kMaxStationsMin ||
        settings.ap.max_stations > kMaxStationsMax) {
        return ConfigError::InvalidMaxStations;
    }
    return ConfigError::None;
}

FieldMerge mergeSsid(AccessPointConfig &config, bool kconfig_valid,
                     NvsValue value, const char *text) noexcept
{
    FieldMerge merge;
    merge.kconfig_invalid = !kconfig_valid;
    if (value != NvsValue::Present || text == nullptr) {
        return merge;
    }
    if (validateSsid(text) != ConfigError::None) {
        return merge;
    }
    if (!setSsid(config, text)) {
        return merge;
    }
    merge.applied = true;
    merge.kconfig_invalid = false;
    return merge;
}

FieldMerge mergePassword(AccessPointConfig &config, bool kconfig_valid,
                         NvsValue value, const char *text) noexcept
{
    FieldMerge merge;
    merge.kconfig_invalid = !kconfig_valid;
    if (value != NvsValue::Present || text == nullptr) {
        return merge;
    }
    // Пустая строка допустима: открытая сеть.
    if (validatePassword(text) != ConfigError::None) {
        return merge;
    }
    if (!setPassword(config, text)) {
        return merge;
    }
    merge.applied = true;
    merge.kconfig_invalid = false;
    return merge;
}

const char *errorName(ConfigError error) noexcept
{
    switch (error) {
        case ConfigError::None:
            return "none";
        case ConfigError::EmptySsid:
            return "empty_ssid";
        case ConfigError::SsidTooLong:
            return "ssid_too_long";
        case ConfigError::SsidNotTerminated:
            return "ssid_not_terminated";
        case ConfigError::SsidNotPrintable:
            return "ssid_not_printable";
        case ConfigError::PasswordTooShort:
            return "password_too_short";
        case ConfigError::PasswordTooLong:
            return "password_too_long";
        case ConfigError::PasswordNotTerminated:
            return "password_not_terminated";
        case ConfigError::PasswordNotPrintable:
            return "password_not_printable";
        case ConfigError::InvalidChannel:
            return "invalid_channel";
        case ConfigError::InvalidMaxStations:
            return "invalid_max_stations";
    }
    return "unknown";
}

bool setSsid(AccessPointConfig &config, const char *ssid) noexcept
{
    if (ssid == nullptr) {
        return false;
    }
    const std::size_t length = boundedLength(ssid, kSsidMaxLength + 1);
    if (length > kSsidMaxLength || !allPrintable(ssid, length)) {
        return false;
    }
    copyString(config.ssid, ssid, length);
    return true;
}

bool setPassword(AccessPointConfig &config, const char *password) noexcept
{
    if (password == nullptr) {
        return false;
    }
    const std::size_t length = boundedLength(password, kPasswordMaxLength + 1);
    if (length > kPasswordMaxLength || !allPrintable(password, length)) {
        return false;
    }
    copyString(config.password, password, length);
    return true;
}

}  // namespace wifi
