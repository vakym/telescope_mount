# Компонент main — точка входа прошивки.
#
# Здесь нет прикладной логики: main инициализирует плату, собирает
# конфигурацию осей и запускает службу движения. Драйверы и решения о
# движении живут в components/.

idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES board motion motor_hal input input_hal wifi_hal
)
