# Компонент state_hal — публикация наблюдаемых снимков.
#
# Здесь разрешены FreeRTOS и esp_timer: назначение компонента — синхронизация
# и публикация. Решения о движении и логика состояния живут в чистых
# компонентах state и control.

idf_component_register(
    SRCS
        "src/state_hub.cpp"
    INCLUDE_DIRS "include"
    REQUIRES state motion input esp_timer
)
