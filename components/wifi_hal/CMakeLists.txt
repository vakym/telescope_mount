# Компонент wifi_hal — NVS-хранилище настроек и служба точки доступа.
#
# Не зависит от motion, motor_hal, input и input_hal: служба только управляет
# радио и публикует состояние. Связывание с кнопкой выполняет main.

idf_component_register(
    SRCS
        "src/settings_store.cpp"
        "src/wifi_ap_service.cpp"
    INCLUDE_DIRS "include"
    REQUIRES wifi esp_wifi esp_netif esp_event nvs_flash
)
