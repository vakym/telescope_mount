# Компонент input_hal — съём ADC1 и GPIO кнопки, задача опроса.
#
# Не зависит от motion и motor_hal: джойстик только публикует положение стика
# и события кнопки. Все алгоритмы обработки находятся в компоненте input.

idf_component_register(
    SRCS
        "src/adc_joystick.cpp"
        "src/button_input.cpp"
        "src/joystick_service.cpp"
    INCLUDE_DIRS "include"
    # esp_driver_gpio: driver/gpio.h подключается из публичного button_input.hpp,
    # поэтому требование публичное.
    REQUIRES board input driver esp_adc esp_timer esp_driver_gpio
)
