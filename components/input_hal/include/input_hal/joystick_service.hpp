/**
 * @file joystick_service.hpp
 * @brief Задача опроса джойстика и потокобезопасный снимок состояния.
 *
 * Сервис не зависит от motor_hal: он только публикует положение стика и
 * события кнопки. Интеграция в контур управления — следующий этап.
 */

#pragma once

#include <cstdint>

#include "board/board.hpp"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "input/button_debouncer.hpp"
#include "input/joystick_config.hpp"
#include "input/joystick_processor.hpp"
#include "input/sample_gate.hpp"
#include "input_hal/adc_joystick.hpp"
#include "input_hal/button_input.hpp"

namespace input_hal {

struct JoystickStatus {
    bool calibrated = false;
    bool sample_valid = false;   ///< результат текущего цикла
    bool output_valid = false;   ///< положение пригодно к использованию
    bool read_fault = false;     ///< подтверждённая серия ошибок
    /// Всегда равно stick.neutral при публикации.
    bool neutral = true;
    input::RawSample raw{};
    input::StickState stick{};
    std::uint64_t last_valid_us = 0;
    std::uint32_t read_errors = 0;
    std::uint32_t button_events = 0;
};

class JoystickService {
public:
    /**
     * @brief Создаёт статические мьютексы и семафор завершения.
     *
     * Контракт владения: создание и первый запуск выполняются до публикации
     * объекта другим задачам. Жизненным циклом управляет одна задача-владелец.
     */
    JoystickService();

    /// Требует согласованной остановки: при таймауте не освобождает объект
    /// с живой задачей и ждёт подтверждения, как AxisMotor.
    ~JoystickService();

    JoystickService(const JoystickService &) = delete;
    JoystickService &operator=(const JoystickService &) = delete;

    /**
     * @brief Запускает опрос.
     *
     * Синхронный: при включённой калибровке может блокировать вызывающую
     * задачу до CONFIG_JOY_CALIBRATION_TIMEOUT_MS.
     */
    esp_err_t start(const board::JoystickPins &pins,
                    input::ButtonListener *listener = nullptr);

    /// Останавливает задачу до освобождения ADC unit и GPIO.
    esp_err_t stop();

    /// Потокобезопасный снимок последнего состояния.
    JoystickStatus status() const;

private:
    enum class Lifecycle : std::uint8_t { Stopped, Running, Stopping };

    static void taskEntry(void *arg);
    void run();
    void publishStatus();
    void lockApi() const;
    void unlockApi() const;

    bool calibrateCenters(input::JoystickConfig &config);
    esp_err_t startFailure(esp_err_t error);

    AdcJoystick adc_{};
    ButtonInput button_{};
    input::JoystickProcessor processor_{};
    input::SampleGate gate_{};
    input::ButtonDebouncer debouncer_{};
    input::ButtonListener *listener_ = nullptr;

    TaskHandle_t task_ = nullptr;
    TickType_t period_ticks_ = 1;
    Lifecycle lifecycle_ = Lifecycle::Stopped;

    /// api_mutex_ сериализует start/stop, status_mutex_ защищает снимок.
    StaticSemaphore_t api_mutex_storage_{};
    SemaphoreHandle_t api_mutex_ = nullptr;
    StaticSemaphore_t status_mutex_storage_{};
    SemaphoreHandle_t status_mutex_ = nullptr;
    StaticSemaphore_t exit_sem_storage_{};
    SemaphoreHandle_t exit_sem_ = nullptr;

    /// local_ изменяет только задача опроса; published_ защищён мьютексом.
    JoystickStatus local_{};
    JoystickStatus published_{};
};

}  // namespace input_hal
