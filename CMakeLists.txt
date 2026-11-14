# Корневой проект прошивки контроллера монтировки телескопа.
#
# Сборка выполняется только через idf.py (обёртка над CMake + ninja), который
# подставляет переменные окружения ESP-IDF и генерирует sdkconfig на основе
# Kconfig компонентов. Прямой вызов cmake из этой папки не поддерживается.

cmake_minimum_required(VERSION 3.16)

# Единый стандарт C++ для всех компонентов проекта.
# Задаётся до project(), чтобы унаследовать значение во всех подкаталогах.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(telescope_mount)
