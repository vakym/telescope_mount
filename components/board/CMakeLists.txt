# Компонент board — конфигурация платы.
#
# Единственное место в проекте, где встречаются номера GPIO. Остальные
# компоненты получают линии из board::kMotorPins и работают с ними как с
# логическими сигналами IN1..IN4.

idf_component_register(
    SRCS "src/board.cpp"
    INCLUDE_DIRS "include"
    # esp_driver_gpio: driver/gpio.h (gpio_num_t, GPIO_NUM_*, gpio_config_t).
    # esp_hal_ana_conv: hal/adc_types.h (adc_unit_t, adc_channel_t).
    # Оба заголовка подключаются из публичного board.hpp, поэтому REQUIRES,
    # а не PRIV_REQUIRES.
    REQUIRES driver hal esp_driver_gpio esp_hal_ana_conv
)
