/**
 * @file settings_store.hpp
 * @brief Настройки точки доступа: Kconfig-значения по умолчанию и NVS.
 *
 * Приоритет строгий: валидный ключ NVS переопределяет Kconfig,
 * отсутствующий ключ замещается Kconfig, невалидный игнорируется с
 * предупреждением. Итоговые настройки обязательно проверяются
 * wifi::validate() вызывающим.
 *
 * save() пишет в NVS. Вызывать его допустимо только при неподвижных осях:
 * запись в flash во время движения запрещена правилами проекта. Сам
 * WifiApService в NVS не пишет.
 */

#pragma once

#include <cstdint>

#include "esp_err.h"
#include "wifi/ap_config.hpp"

namespace wifi_hal {

enum class SettingSource : std::uint8_t {
    KconfigDefault,
    Nvs,
};

enum class SettingIssue : std::uint8_t {
    None,           ///< ключ отсутствует — использован Kconfig
    InvalidValue,   ///< ключ есть, но значение отвергнуто
    ReadError,      ///< ошибка чтения NVS
    /// Значение Kconfig не помещается в буфер или непечатаемо. Не
    /// обрезается: запуск возможен только при валидном переопределении
    /// этого поля из NVS.
    KconfigInvalid,
};

struct FieldStatus {
    SettingSource source = SettingSource::KconfigDefault;
    SettingIssue issue = SettingIssue::None;
    esp_err_t error = ESP_OK;
};

struct LoadResult {
    esp_err_t status = ESP_OK;
    wifi::AccessPointSettings settings{};
    FieldStatus ssid{};
    FieldStatus password{};
    FieldStatus channel{};
    FieldStatus max_stations{};
    FieldStatus hidden{};
    FieldStatus start_on_boot{};
    bool any_invalid = false;
    bool any_read_error = false;
};

class SettingsStore {
public:
    /**
     * @brief Инициализация NVS, один раз за время работы приложения.
     *
     * При ESP_ERR_NVS_NO_FREE_PAGES или ESP_ERR_NVS_NEW_VERSION_FOUND
     * стирается весь используемый NVS-раздел (не только namespace wifi):
     * это происходит только на старте и никогда не повторяется по нажатию
     * кнопки.
     */
    static esp_err_t initNvs();

    /// Kconfig defaults + переопределение из NVS. В flash не пишет.
    LoadResult load() const;

    /// Запись настроек. Атомарность набора ключей не гарантируется.
    esp_err_t save(const wifi::AccessPointSettings &settings);
};

}  // namespace wifi_hal
