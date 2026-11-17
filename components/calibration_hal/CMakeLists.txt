# Компонент calibration_hal — ESP-IDF адаптер калибровки масштаба.
#
# Здесь только задачи, NVS и адаптер ControlService. Расчёт и автомат сессии
# живут в чистом components/calibration.

idf_component_register(
    SRCS
        "src/control_port.cpp"
        "src/nvs_store.cpp"
        "src/calibration_service.cpp"
    INCLUDE_DIRS "include"
    REQUIRES calibration control_hal state_hal motor_hal nvs_flash
)
