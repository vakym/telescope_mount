# Компонент board — конфигурация платы.
#
# Единственное место в проекте, где встречаются номера GPIO. Остальные
# компоненты получают линии из board::kMotorPins и работают с ними как с
# логическими сигналами IN1..IN4.

idf_component_register(
    SRCS "src/board.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver hal
)
