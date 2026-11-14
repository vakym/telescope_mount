/**
 * @file main.cpp
 * @brief Точка входа прошивки контроллера монтировки телескопа.
 *
 * Этап 1: минимальный каркас ESP-IDF.
 *
 * Ограничения этапа:
 *  - драйверы двигателей не реализованы;
 *  - обработчики прерываний не реализованы;
 *  - номера GPIO не назначаются (единственный источник — компонент board,
 *    который появится после подтверждения распиновки).
 *
 * Поэтому app_main только сообщает о запуске, печатает сведения о кристалле и
 * остаётся живым, чтобы в мониторе был виден heartbeat.
 */

#include <cstdint>

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char *kTag = "telescope_mount";

/// Период heartbeat в мониторе, мс.
constexpr std::uint32_t kHeartbeatPeriodMs = 5000;

void logChipInfo()
{
    esp_chip_info_t chip_info{};
    esp_chip_info(&chip_info);
    ESP_LOGI(kTag, "chip: %d core(s), silicon revision %d",
             chip_info.cores, chip_info.revision);

    std::uint32_t flash_size_bytes = 0;
    if (esp_flash_get_size(nullptr, &flash_size_bytes) == ESP_OK) {
        ESP_LOGI(kTag, "flash: %u MB",
                 static_cast<unsigned>(flash_size_bytes / (1024U * 1024U)));
    } else {
        ESP_LOGW(kTag, "flash: size unknown");
    }
}

}  // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(kTag, "firmware starting");
    ESP_LOGI(kTag, "ESP-IDF version: %s", esp_get_idf_version());

    logChipInfo();
    ESP_LOGI(kTag, "stage 1: skeleton only, motor driver and GPIO are not implemented");

    // Прикладных задач пока нет: держим задачу app_main живой,
    // чтобы подтвердить успешный запуск прошивки по heartbeat в мониторе.
    while (true) {
        ESP_LOGI(kTag, "heartbeat");
        vTaskDelay(pdMS_TO_TICKS(kHeartbeatPeriodMs));
    }
}
