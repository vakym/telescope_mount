# Компонент web_hal — локальный HTTP-сервер состояния.
#
# Зависимости направлены только внутрь чтения: StateHub (снимок состояния) и
# WifiApService (подтверждённое состояние радио). Моторных слоёв здесь нет, и
# wifi_hal о существовании HTTP не знает.
#
# Страница встроена в образ через EMBED_FILES: отдельная файловая система не
# добавляется. Символы _binary_index_html_start/_end доступны только внутри
# этого компонента.

# При выключенной опции компонент не содержит ни кода сервера, ни страницы:
# это и аварийный откат, и честное сравнение размеров образа.
if(CONFIG_WEB_ENABLED)
    set(web_sources "src/http_status_service.cpp")
    set(web_embeds "www/index.html")
else()
    set(web_sources "")
    set(web_embeds "")
endif()

idf_component_register(
    SRCS ${web_sources}
    INCLUDE_DIRS "include"
    EMBED_FILES ${web_embeds}
    REQUIRES web state state_hal wifi_hal calibration_hal esp_http_server esp_timer
)
