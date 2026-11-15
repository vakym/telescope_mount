# Компонент control_hal — управляющая задача и адаптер исполнения.
#
# Единственный исполнитель моторных операций: команды в MotorService идут
# только отсюда и только из задачи. Внешние вызовы ставят запросы.

idf_component_register(
    SRCS
        "src/motor_axis_port.cpp"
        "src/external_intents.cpp"
        "src/control_service.cpp"
    INCLUDE_DIRS "include"
    REQUIRES control state state_hal motor_hal input_hal esp_timer
)
