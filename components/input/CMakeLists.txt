# Компонент input — чистая обработка положений стика и кнопки.
#
# Не зависит от ESP-IDF, FreeRTOS и motor_hal. Благодаря этому исходники
# подключаются к обычному CMake-проекту test/host и покрываются тестами без
# железа.

idf_component_register(
    SRCS
        "src/joystick_processor.cpp"
        "src/sample_gate.cpp"
        "src/button_debouncer.cpp"
    INCLUDE_DIRS "include"
)
