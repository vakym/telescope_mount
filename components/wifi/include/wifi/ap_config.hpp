/**
 * @file ap_config.hpp
 * @brief Параметры точки доступа и их валидация.
 *
 * Компонент wifi — чистая логика C++17 без зависимости от ESP-IDF.
 * Значения по умолчанию проекта (`mount_01`, `12345678`, канал 1) задаются
 * в Kconfig и собираются в wifi_hal: здесь они не дублируются.
 *
 * Буферы публичны, поэтому валидация обязана работать и с данными,
 * заполненными не через сеттеры: отсутствие терминатора внутри массива —
 * отдельная ошибка длины, а не чтение за границы.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace wifi {

constexpr std::size_t kSsidMaxLength = 32;
constexpr std::size_t kPasswordMaxLength = 63;
constexpr std::size_t kSsidCapacity = kSsidMaxLength + 1;
constexpr std::size_t kPasswordCapacity = kPasswordMaxLength + 1;
constexpr std::uint8_t kChannelMin = 1;
/// Каналы 12-13 требуют настройки country и отложены.
constexpr std::uint8_t kChannelMax = 11;
constexpr std::uint8_t kMaxStationsMin = 1;
constexpr std::uint8_t kMaxStationsMax = 4;

/// Параметры точки доступа.
struct AccessPointConfig {
    char ssid[kSsidCapacity] = {};
    char password[kPasswordCapacity] = {};
    std::uint8_t channel = kChannelMin;
    std::uint8_t max_stations = 1;
    bool hidden = false;
};

/// Настройки целиком: параметры и автозапуск при включении.
struct AccessPointSettings {
    AccessPointConfig ap{};
    bool start_on_boot = true;
};

enum class ConfigError : std::uint8_t {
    None,
    EmptySsid,
    SsidTooLong,
    SsidNotTerminated,
    SsidNotPrintable,
    PasswordTooShort,
    PasswordTooLong,
    PasswordNotTerminated,
    PasswordNotPrintable,
    InvalidChannel,
    InvalidMaxStations,
};

/**
 * @brief Проверяет настройки перед запуском.
 *
 * Пустой пароль допустим и означает открытую сеть. Невалидный итоговый
 * конфиг — ошибка инициализации, а не повод молча подставить значения.
 */
ConfigError validate(const AccessPointSettings &settings) noexcept;

/// Происхождение значения поля из NVS.
enum class NvsValue : std::uint8_t {
    Absent,  ///< ключа нет — используется значение Kconfig
    Invalid, ///< значение есть, но непригодно
    Present, ///< значение есть и структурно корректно
};

/// Результат применения значения NVS к полю.
struct FieldMerge {
    bool applied = false;         ///< значение NVS применено
    bool kconfig_invalid = false; ///< значение Kconfig непригодно
};

/**
 * @brief Применяет значение NVS к SSID.
 *
 * Признак непригодного значения Kconfig снимается только после успешного
 * применения валидного значения NVS: невалидное NVS его не скрывает.
 */
FieldMerge mergeSsid(AccessPointConfig &config, bool kconfig_valid,
                     NvsValue value, const char *text) noexcept;

/**
 * @brief Применяет значение NVS к паролю.
 *
 * Пустая строка — намеренный выбор открытой сети и успешно переопределяет
 * непригодное значение Kconfig.
 */
FieldMerge mergePassword(AccessPointConfig &config, bool kconfig_valid,
                         NvsValue value, const char *text) noexcept;

/// Имя ошибки для лога; для ConfigError::None возвращает "none".
const char *errorName(ConfigError error) noexcept;

/**
 * @brief Записывает SSID.
 *
 * nullptr, строка длиннее 32 символов или непечатный символ — false,
 * объект не изменяется.
 */
bool setSsid(AccessPointConfig &config, const char *ssid) noexcept;

/**
 * @brief Записывает пароль.
 *
 * nullptr, строка длиннее 63 символов или непечатный символ — false,
 * объект не изменяется. Пустая строка допустима (открытая сеть).
 */
bool setPassword(AccessPointConfig &config, const char *password) noexcept;

}  // namespace wifi
