/**
 * @file nvs_store.hpp
 * @brief Постоянное хранение коэффициентов калибровки в NVS.
 *
 * Две записи на ось (слот A/B). Читается валидная запись с максимальным
 * sequence, пишется всегда в слот с меньшим sequence. Запись выполняется
 * только при открытом flash-окне: движение в этот момент запрещено
 * управляющим слоем.
 */

#pragma once

#include <cstdint>

#include "calibration/record.hpp"
#include "esp_err.h"
#include "motion/types.hpp"

namespace calibration_hal {

class NvsStore {
public:
    struct Loaded {
        bool present = false;
        esp_err_t error = ESP_OK;
        calibration::Record record{};
    };

    /// Открывает namespace. Повторный вызов безопасен.
    esp_err_t init();

    /// Читает оба слота оси. Ошибка NVS не маскируется «пустой» записью.
    Loaded load(motion::AxisId id);

    /// Пишет запись в слот с меньшим sequence и увеличивает sequence.
    /// Требует открытого flash-окна у вызывающего.
    esp_err_t save(motion::AxisId id, calibration::Record record);

    /// Следующий sequence, который будет назначен при сохранении.
    std::uint32_t nextSequence() const noexcept { return next_sequence_; }

private:
    static const char *keyOf(motion::AxisId id, std::uint8_t slot);
    static esp_err_t readSlot(const char *key, calibration::Record &out,
                              calibration::RecordResult &result);
    static esp_err_t writeSlot(const char *key, const calibration::Record &record);

    bool ready_ = false;
    std::uint32_t next_sequence_ = 1;
};

}  // namespace calibration_hal
