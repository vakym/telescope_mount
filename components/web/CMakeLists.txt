# Компонент web — контракт веб-слоя, сериализация и автомат жизненного цикла.
#
# Чистая логика C++17 без ESP-IDF: контракт JSON, ограниченный писатель
# документа и автомат запуска/останова сервера. Благодаря этому исходники
# подключаются к test/host и тестируются без железа.
#
# Компонент не зависит от HTTP, FreeRTOS и моторных слоёв: только типы
# наблюдаемого состояния.

idf_component_register(
    SRCS
        "src/state_json.cpp"
        "src/server_lifecycle.cpp"
        "src/server_controller.cpp"
    INCLUDE_DIRS "include"
    REQUIRES state motion input
)
