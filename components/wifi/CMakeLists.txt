# Компонент wifi — чистая логика точки доступа.
#
# Не зависит от ESP-IDF, FreeRTOS и motor_hal. Благодаря этому исходники
# подключаются к обычному CMake-проекту test/host и покрываются тестами без
# железа.

idf_component_register(
    SRCS
        "src/ap_config.cpp"
        "src/toggle_admission.cpp"
        "src/ap_lifecycle.cpp"
    INCLUDE_DIRS "include"
)
