/**
 * @file test_ap_settings_merge.cpp
 * @brief Регрессии слияния значений Kconfig и NVS.
 *
 * Проверяется путь «формирование defaults → применение значения NVS», а не
 * только wifi::validate(): именно там терялся признак непригодного значения
 * Kconfig. Пустой пароль в NVS — намеренный выбор открытой сети; невалидное
 * значение NVS не снимает KconfigInvalid.
 */

#include <cstddef>
#include <cstdint>

#include "check.hpp"
#include "wifi/ap_config.hpp"

namespace {

using wifi::AccessPointSettings;
using wifi::ConfigError;
using wifi::FieldMerge;
using wifi::NvsValue;

constexpr std::size_t kLongPasswordLength = 64;

void fillString(char *buffer, std::size_t length, char value)
{
    for (std::size_t i = 0; i < length; ++i) {
        buffer[i] = value;
    }
    buffer[length] = '\0';
}

bool equals(const char *actual, const char *expected)
{
    std::size_t i = 0;
    while (expected[i] != '\0') {
        if (actual[i] != expected[i]) {
            return false;
        }
        ++i;
    }
    return actual[i] == '\0';
}

struct PasswordResolution {
    AccessPointSettings settings;
    bool kconfig_valid = true;
    FieldMerge merge{};
    ConfigError final_error = ConfigError::None;
};

PasswordResolution resolvePassword(const char *kconfig_password,
                                   NvsValue value, const char *nvs_text)
{
    PasswordResolution resolution;
    CHECK(wifi::setSsid(resolution.settings.ap, "mount_01"));
    resolution.kconfig_valid =
        wifi::setPassword(resolution.settings.ap, kconfig_password);
    if (!resolution.kconfig_valid) {
        // Как в settings_store: непригодное значение не применяется.
        resolution.settings.ap.password[0] = '\0';
    }
    resolution.merge = wifi::mergePassword(resolution.settings.ap,
                                           resolution.kconfig_valid, value,
                                           nvs_text);
    resolution.final_error = wifi::validate(resolution.settings);
    return resolution;
}

/// Служба отвергает запуск, если исходное Kconfig-значение непригодно и не
/// заменено валидным NVS.
bool startAllowed(const PasswordResolution &resolution)
{
    return !resolution.merge.kconfig_invalid &&
           resolution.final_error == ConfigError::None;
}

struct SsidResolution {
    AccessPointSettings settings;
    bool kconfig_valid = true;
    FieldMerge merge{};
    ConfigError final_error = ConfigError::None;
};

SsidResolution resolveSsid(NvsValue value, const char *nvs_text)
{
    SsidResolution resolution;
    char ssid33[wifi::kSsidCapacity + 1]{};
    fillString(ssid33, wifi::kSsidCapacity, 's');  // 33 символа
    resolution.kconfig_valid = wifi::setSsid(resolution.settings.ap, ssid33);
    CHECK(!resolution.kconfig_valid);
    CHECK(wifi::setPassword(resolution.settings.ap, "12345678"));
    resolution.merge = wifi::mergeSsid(resolution.settings.ap,
                                       resolution.kconfig_valid, value,
                                       nvs_text);
    resolution.final_error = wifi::validate(resolution.settings);
    return resolution;
}

bool startAllowed(const SsidResolution &resolution)
{
    return !resolution.merge.kconfig_invalid &&
           resolution.final_error == ConfigError::None;
}

void longKconfigPasswordWithoutOverride()
{
    char long_password[kLongPasswordLength + 1]{};
    fillString(long_password, kLongPasswordLength, 'p');

    const PasswordResolution resolution =
        resolvePassword(long_password, NvsValue::Absent, "");
    CHECK(!resolution.kconfig_valid);
    CHECK(resolution.merge.kconfig_invalid);
    // validate() сам по себе проблемы не видит: пустое поле выглядит как
    // открытая сеть. Решение принимает признак KconfigInvalid.
    CHECK(resolution.final_error == ConfigError::None);
    CHECK(!startAllowed(resolution));
}

void longKconfigPasswordWrongNvsType()
{
    char long_password[kLongPasswordLength + 1]{};
    fillString(long_password, kLongPasswordLength, 'p');

    const PasswordResolution resolution =
        resolvePassword(long_password, NvsValue::Invalid, nullptr);
    CHECK(resolution.merge.kconfig_invalid);
    CHECK(!resolution.merge.applied);
    CHECK(!startAllowed(resolution));
}

void longKconfigPasswordShortNvs()
{
    char long_password[kLongPasswordLength + 1]{};
    fillString(long_password, kLongPasswordLength, 'p');

    const PasswordResolution resolution =
        resolvePassword(long_password, NvsValue::Present, "12345");
    CHECK(resolution.merge.kconfig_invalid);
    CHECK(!resolution.merge.applied);
    CHECK(!startAllowed(resolution));
}

void longKconfigPasswordTooLongNvs()
{
    char long_password[kLongPasswordLength + 1]{};
    fillString(long_password, kLongPasswordLength, 'p');
    char long_nvs[kLongPasswordLength + 1]{};
    fillString(long_nvs, kLongPasswordLength, 'n');

    const PasswordResolution resolution =
        resolvePassword(long_password, NvsValue::Present, long_nvs);
    CHECK(resolution.merge.kconfig_invalid);
    CHECK(!resolution.merge.applied);
    CHECK(!startAllowed(resolution));
}

void longKconfigPasswordValidNvs()
{
    char long_password[kLongPasswordLength + 1]{};
    fillString(long_password, kLongPasswordLength, 'p');

    const PasswordResolution resolution =
        resolvePassword(long_password, NvsValue::Present, "87654321");
    CHECK(resolution.merge.applied);
    CHECK(!resolution.merge.kconfig_invalid);
    CHECK(startAllowed(resolution));
    CHECK(equals(resolution.settings.ap.password, "87654321"));
}

void longKconfigPasswordExplicitOpenNetwork()
{
    char long_password[kLongPasswordLength + 1]{};
    fillString(long_password, kLongPasswordLength, 'p');

    // Явно сохранённая пустая строка — намеренная открытая сеть.
    const PasswordResolution resolution =
        resolvePassword(long_password, NvsValue::Present, "");
    CHECK(resolution.merge.applied);
    CHECK(!resolution.merge.kconfig_invalid);
    CHECK(startAllowed(resolution));
    CHECK(resolution.settings.ap.password[0] == '\0');
}

void validKconfigPasswordInvalidNvs()
{
    const PasswordResolution resolution =
        resolvePassword("12345678", NvsValue::Invalid, nullptr);
    CHECK(!resolution.merge.applied);
    CHECK(!resolution.merge.kconfig_invalid);
    CHECK(startAllowed(resolution));
    CHECK(equals(resolution.settings.ap.password, "12345678"));
}

void longKconfigSsidWithoutOverride()
{
    const SsidResolution resolution = resolveSsid(NvsValue::Absent, "");
    CHECK(resolution.merge.kconfig_invalid);
    CHECK(!startAllowed(resolution));
}

void longKconfigSsidValidNvs()
{
    const SsidResolution resolution =
        resolveSsid(NvsValue::Present, "nvs_network");
    CHECK(resolution.merge.applied);
    CHECK(!resolution.merge.kconfig_invalid);
    CHECK(startAllowed(resolution));
    CHECK(equals(resolution.settings.ap.ssid, "nvs_network"));
}

void longKconfigSsidInvalidNvs()
{
    const SsidResolution resolution =
        resolveSsid(NvsValue::Invalid, nullptr);
    CHECK(!resolution.merge.applied);
    CHECK(resolution.merge.kconfig_invalid);
    CHECK(!startAllowed(resolution));
}

}  // namespace

int main()
{
    longKconfigPasswordWithoutOverride();
    longKconfigPasswordWrongNvsType();
    longKconfigPasswordShortNvs();
    longKconfigPasswordTooLongNvs();
    longKconfigPasswordValidNvs();
    longKconfigPasswordExplicitOpenNetwork();
    validKconfigPasswordInvalidNvs();
    longKconfigSsidWithoutOverride();
    longKconfigSsidValidNvs();
    longKconfigSsidInvalidNvs();
    return TEST_RESULT();
}
