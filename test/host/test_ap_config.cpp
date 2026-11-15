/**
 * @file test_ap_config.cpp
 * @brief Проверки строковых контрактов и валидации точки доступа.
 *
 * Отдельно разделены сценарии, которые легко перепутать:
 *  - внешняя строка длиной 33 или 64 отвергается сеттером;
 *  - заполненный массив без терминатора даёт отдельную ошибку длины;
 *  - строка ровно на границе ёмкости валидна.
 */

#include <cstddef>
#include <cstdint>

#include "check.hpp"
#include "wifi/ap_config.hpp"

namespace {

using wifi::AccessPointSettings;
using wifi::ConfigError;

AccessPointSettings makeValid()
{
    AccessPointSettings settings;
    CHECK(wifi::setSsid(settings.ap, "mount_01"));
    CHECK(wifi::setPassword(settings.ap, "12345678"));
    return settings;
}

void defaults()
{
    AccessPointSettings settings{};
    CHECK(settings.start_on_boot);
    CHECK(settings.ap.channel == 1);
    CHECK(settings.ap.max_stations == 1);
    CHECK(!settings.ap.hidden);
    CHECK(settings.ap.ssid[0] == '\0');
    // Пустой SSID по умолчанию невалиден: значения проекта задаёт Kconfig.
    CHECK(wifi::validate(settings) == ConfigError::EmptySsid);
}

void validSettings()
{
    CHECK(wifi::validate(makeValid()) == ConfigError::None);
}

void ssidBoundaries()
{
    AccessPointSettings exact = makeValid();
    for (std::size_t i = 0; i < wifi::kSsidMaxLength; ++i) {
        exact.ap.ssid[i] = 'a';
    }
    exact.ap.ssid[wifi::kSsidMaxLength] = '\0';
    CHECK(wifi::validate(exact) == ConfigError::None);

    AccessPointSettings unterminated = makeValid();
    for (std::size_t i = 0; i < wifi::kSsidCapacity; ++i) {
        unterminated.ap.ssid[i] = 'a';
    }
    CHECK(wifi::validate(unterminated) == ConfigError::SsidNotTerminated);
}

void setSsidRejectsLongInput()
{
    AccessPointSettings settings = makeValid();
    const char *too_long = "123456789012345678901234567890123";  // 33 символа
    CHECK(!wifi::setSsid(settings.ap, too_long));
    CHECK(settings.ap.ssid[0] == 'm');
    CHECK(wifi::validate(settings) == ConfigError::None);
}

void ssidNotPrintable()
{
    AccessPointSettings control = makeValid();
    control.ap.ssid[0] = 'a';
    control.ap.ssid[1] = 0x1F;
    control.ap.ssid[2] = '\0';
    CHECK(wifi::validate(control) == ConfigError::SsidNotPrintable);

    AccessPointSettings del = makeValid();
    del.ap.ssid[0] = 'a';
    del.ap.ssid[1] = 0x7F;
    del.ap.ssid[2] = '\0';
    CHECK(wifi::validate(del) == ConfigError::SsidNotPrintable);
}

void passwordRules()
{
    AccessPointSettings settings = makeValid();

    CHECK(wifi::setPassword(settings.ap, ""));
    CHECK(wifi::validate(settings) == ConfigError::None);  // открытая сеть

    CHECK(wifi::setPassword(settings.ap, "1234567"));
    CHECK(wifi::validate(settings) == ConfigError::PasswordTooShort);

    CHECK(wifi::setPassword(settings.ap, "12345678"));
    CHECK(wifi::validate(settings) == ConfigError::None);

    char password63[wifi::kPasswordCapacity]{};
    for (std::size_t i = 0; i < wifi::kPasswordMaxLength; ++i) {
        password63[i] = 'p';
    }
    CHECK(wifi::setPassword(settings.ap, password63));
    CHECK(wifi::validate(settings) == ConfigError::None);
}

void setPasswordRejectsLongInput()
{
    AccessPointSettings settings = makeValid();
    char password64[wifi::kPasswordCapacity + 1]{};
    for (std::size_t i = 0; i < wifi::kPasswordCapacity; ++i) {
        password64[i] = 'p';
    }
    CHECK(!wifi::setPassword(settings.ap, password64));
    // Прежний пароль сохранился.
    CHECK(settings.ap.password[0] == '1');
    CHECK(wifi::validate(settings) == ConfigError::None);
}

void passwordNotTerminated()
{
    AccessPointSettings settings = makeValid();
    for (std::size_t i = 0; i < wifi::kPasswordCapacity; ++i) {
        settings.ap.password[i] = 'p';
    }
    CHECK(wifi::validate(settings) == ConfigError::PasswordNotTerminated);
}

void passwordNotPrintable()
{
    AccessPointSettings settings = makeValid();
    for (std::size_t i = 0; i < 7; ++i) {
        settings.ap.password[i] = static_cast<char>('1' + i);
    }
    settings.ap.password[7] = static_cast<char>(0x7F);
    settings.ap.password[8] = '\0';
    CHECK(wifi::validate(settings) == ConfigError::PasswordNotPrintable);
}

void channelValidation()
{
    AccessPointSettings settings = makeValid();
    settings.ap.channel = 0;
    CHECK(wifi::validate(settings) == ConfigError::InvalidChannel);
    settings.ap.channel = 12;
    CHECK(wifi::validate(settings) == ConfigError::InvalidChannel);
    settings.ap.channel = 1;
    CHECK(wifi::validate(settings) == ConfigError::None);
    settings.ap.channel = 11;
    CHECK(wifi::validate(settings) == ConfigError::None);
}

void stationsValidation()
{
    AccessPointSettings settings = makeValid();
    settings.ap.max_stations = 0;
    CHECK(wifi::validate(settings) == ConfigError::InvalidMaxStations);
    settings.ap.max_stations = 5;
    CHECK(wifi::validate(settings) == ConfigError::InvalidMaxStations);
    settings.ap.max_stations = 1;
    CHECK(wifi::validate(settings) == ConfigError::None);
    settings.ap.max_stations = 4;
    CHECK(wifi::validate(settings) == ConfigError::None);
}

void nullptrSetters()
{
    wifi::AccessPointConfig config;
    CHECK(!wifi::setSsid(config, nullptr));
    CHECK(!wifi::setPassword(config, nullptr));
    CHECK(config.ssid[0] == '\0');
    CHECK(config.password[0] == '\0');
}

void setterSuccess()
{
    wifi::AccessPointConfig config;
    CHECK(wifi::setSsid(config, "abc"));
    CHECK(config.ssid[0] == 'a');
    CHECK(config.ssid[3] == '\0');
    CHECK(wifi::setPassword(config, ""));
    CHECK(config.password[0] == '\0');
}

void namesForAllErrors()
{
    const ConfigError errors[] = {
        ConfigError::None,
        ConfigError::EmptySsid,
        ConfigError::SsidTooLong,
        ConfigError::SsidNotTerminated,
        ConfigError::SsidNotPrintable,
        ConfigError::PasswordTooShort,
        ConfigError::PasswordTooLong,
        ConfigError::PasswordNotTerminated,
        ConfigError::PasswordNotPrintable,
        ConfigError::InvalidChannel,
        ConfigError::InvalidMaxStations,
    };
    for (const ConfigError error : errors) {
        const char *name = wifi::errorName(error);
        CHECK(name != nullptr);
        CHECK(name[0] != '\0');
    }
}

}  // namespace

int main()
{
    defaults();
    validSettings();
    ssidBoundaries();
    setSsidRejectsLongInput();
    ssidNotPrintable();
    passwordRules();
    setPasswordRejectsLongInput();
    passwordNotTerminated();
    passwordNotPrintable();
    channelValidation();
    stationsValidation();
    nullptrSetters();
    setterSuccess();
    namesForAllErrors();
    return TEST_RESULT();
}
