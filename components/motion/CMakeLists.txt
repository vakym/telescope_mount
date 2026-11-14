# Компонент motion — чистая логика движения.
#
# Не зависит от ESP-IDF: ни драйверов, ни FreeRTOS. Благодаря этому исходники
# подключаются к обычному CMake-проекту test/host и покрываются тестами без
# железа.

idf_component_register(
    SRCS
        "src/axis_config.cpp"
        "src/axis_command.cpp"
        "src/axis_planner.cpp"
        "src/deadline_policy.cpp"
        "src/axis_executor.cpp"
        "src/motion_controller.cpp"
    INCLUDE_DIRS "include"
)
