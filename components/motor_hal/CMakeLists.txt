# Компонент motor_hal — исполнение: GPIO, GPTimer, задачи FreeRTOS.
#
# Решения о движении принимает motion::AxisExecutor; здесь только доставка
# событий и запись фаз. Драйверы GPTimer входят в состав компонента driver.

idf_component_register(
    SRCS
        "src/coil_driver.cpp"
        "src/gptimer_step_clock.cpp"
        "src/axis_motor.cpp"
        "src/motor_service.cpp"
    INCLUDE_DIRS "include"
    REQUIRES board motion driver
)
