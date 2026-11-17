# Компонент main — точка входа прошивки.
#
# Здесь нет прикладной логики: main инициализирует плату, собирает
# конфигурацию осей и запускает службу движения. Драйверы и решения о
# движении живут в components/.

idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    # spi_flash: esp_flash.h; esp_timer: esp_timer.h — использует main.cpp.
    REQUIRES
        board motion motor_hal input input_hal wifi_hal state state_hal control control_hal
        calibration calibration_hal web_hal
        esp_timer spi_flash
)
