/**
 * @file board.hpp
 * @brief Конфигурация платы: единственное место с номерами GPIO.
 *
 * Источник истины по распиновке — docs/hardware/pinmap.md. Изменение проводки
 * сначала вносится в документ, затем в этот файл. Другие компоненты работают
 * только с логическими именами сигналов.
 */

#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

namespace board {

/// Четыре линии ULN2003: IN1..IN4.
struct CoilPins {
    gpio_num_t in[4];
};

/// Группы линий обеих осей.
struct MotorPins {
    CoilPins azimuth;
    CoilPins altitude;
};

/// Подтверждённая распиновка стенда (docs/hardware/pinmap.md).
constexpr MotorPins kMotorPins = {
    {{GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14}},  // азимут
    {{GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_23, GPIO_NUM_13}},  // высота
};

/**
 * @brief Переводит все 8 линий INx в выход с низким уровнем.
 *
 * Обязательный первый шаг инициализации: сразу после сброса GPIO находятся в
 * высокоимпедансном состоянии, а обмотки должны быть гарантированно
 * выключены. Вызывается до создания задач и любых команд движения.
 */
esp_err_t init();

}  // namespace board
