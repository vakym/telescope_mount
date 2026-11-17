# Компонент calibration — чистая логика масштаба механики.
#
# Не зависит от ESP-IDF, FreeRTOS и NVS: только типы motion/state и стандартная
# библиотека C++17. Исходники подключаются к test/host.

idf_component_register(
    SRCS
        "src/types.cpp"
        "src/math.cpp"
        "src/session.cpp"
        "src/record.cpp"
    INCLUDE_DIRS "include"
    REQUIRES motion state
)
