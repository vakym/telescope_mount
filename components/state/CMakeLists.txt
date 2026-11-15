# Компонент state — типы наблюдаемого состояния и правила свежести.
#
# Не зависит от ESP-IDF и FreeRTOS: только типы motion и input. Благодаря
# этому исходники подключаются к test/host и тестируются без железа.

idf_component_register(
    SRCS
        "src/health.cpp"
        "src/channels.cpp"
    INCLUDE_DIRS "include"
    REQUIRES motion input
)
