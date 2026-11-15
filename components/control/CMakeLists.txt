# Компонент control — чистая логика управления.
#
# Не зависит от ESP-IDF, FreeRTOS, motor_hal и input_hal: только типы state и
# motion. Благодаря этому исходники подключаются к test/host и тестируются
# без железа.

idf_component_register(
    SRCS
        "src/joy_mapping.cpp"
        "src/joy_observation.cpp"
        "src/input_grace.cpp"
        "src/rate_gate.cpp"
        "src/stop_protocol.cpp"
        "src/arming.cpp"
        "src/intent_inbox.cpp"
        "src/arbiter.cpp"
    INCLUDE_DIRS "include"
    REQUIRES state motion input
)
