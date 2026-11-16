# Host-тесты логики движения: обычный CMake, без idf.py и без железа.
#
#   cmake -S test/host -B build-host -DCMAKE_BUILD_TYPE=Debug
#   cmake --build build-host
#   ctest --test-dir build-host --output-on-failure
#
# Компоненты motion, input и wifi подключаются исходниками: они не зависят
# от ESP-IDF.

cmake_minimum_required(VERSION 3.16)
project(telescope_mount_host_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(ENABLE_SANITIZERS "Build tests with AddressSanitizer and UBSan" OFF)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(MOTION_DIR "${REPO_ROOT}/components/motion")
set(INPUT_DIR "${REPO_ROOT}/components/input")
set(WIFI_DIR "${REPO_ROOT}/components/wifi")
set(STATE_DIR "${REPO_ROOT}/components/state")
set(CONTROL_DIR "${REPO_ROOT}/components/control")
set(WEB_DIR "${REPO_ROOT}/components/web")

add_library(motion_host STATIC
    "${MOTION_DIR}/src/axis_config.cpp"
    "${MOTION_DIR}/src/axis_command.cpp"
    "${MOTION_DIR}/src/axis_planner.cpp"
    "${MOTION_DIR}/src/deadline_policy.cpp"
    "${MOTION_DIR}/src/axis_executor.cpp"
    "${MOTION_DIR}/src/motion_controller.cpp"
)
target_include_directories(motion_host PUBLIC "${MOTION_DIR}/include")
target_compile_options(motion_host PRIVATE -Wall -Wextra -Werror)

add_library(input_host STATIC
    "${INPUT_DIR}/src/joystick_processor.cpp"
    "${INPUT_DIR}/src/sample_gate.cpp"
    "${INPUT_DIR}/src/button_debouncer.cpp"
)
target_include_directories(input_host PUBLIC "${INPUT_DIR}/include")
target_compile_options(input_host PRIVATE -Wall -Wextra -Werror)

add_library(wifi_host STATIC
    "${WIFI_DIR}/src/ap_config.cpp"
    "${WIFI_DIR}/src/toggle_admission.cpp"
    "${WIFI_DIR}/src/ap_lifecycle.cpp"
)
target_include_directories(wifi_host PUBLIC "${WIFI_DIR}/include")
target_compile_options(wifi_host PRIVATE -Wall -Wextra -Werror)

add_library(state_host STATIC
    "${STATE_DIR}/src/health.cpp"
    "${STATE_DIR}/src/channels.cpp"
)
target_include_directories(state_host PUBLIC "${STATE_DIR}/include")
target_link_libraries(state_host PUBLIC motion_host input_host)
target_compile_options(state_host PRIVATE -Wall -Wextra -Werror)

# control зависит от state и типов motion/input; id назначает нижний слой,
# поэтому control_host тестируется с fake-портом исполнения.
add_library(control_host STATIC
    "${CONTROL_DIR}/src/joy_mapping.cpp"
    "${CONTROL_DIR}/src/joy_observation.cpp"
    "${CONTROL_DIR}/src/input_grace.cpp"
    "${CONTROL_DIR}/src/rate_gate.cpp"
    "${CONTROL_DIR}/src/stop_protocol.cpp"
    "${CONTROL_DIR}/src/arming.cpp"
    "${CONTROL_DIR}/src/intent_inbox.cpp"
    "${CONTROL_DIR}/src/arbiter.cpp"
)
target_include_directories(control_host PUBLIC "${CONTROL_DIR}/include")
target_link_libraries(control_host PUBLIC state_host motion_host input_host)
target_compile_options(control_host PRIVATE -Wall -Wextra -Werror)

# web — контракт и автомат веб-слоя: чистый код, тестируется на хосте.
# Сам HTTP-сервер живёт в web_hal и здесь не собирается.
add_library(web_host STATIC
    "${WEB_DIR}/src/state_json.cpp"
    "${WEB_DIR}/src/server_lifecycle.cpp"
    "${WEB_DIR}/src/server_controller.cpp"
)
target_include_directories(web_host PUBLIC "${WEB_DIR}/include")
target_link_libraries(web_host PUBLIC state_host)
target_compile_options(web_host PRIVATE -Wall -Wextra -Werror)

if(ENABLE_SANITIZERS)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

enable_testing()

foreach(test_name
        test_deadline_policy
        test_axis_planner
        test_axis_executor
        test_motion_controller)
    add_executable(${test_name} "${test_name}.cpp")
    target_link_libraries(${test_name} PRIVATE motion_host)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

foreach(test_name
        test_joystick_processor
        test_sample_gate
        test_button_debouncer)
    add_executable(${test_name} "${test_name}.cpp")
    target_link_libraries(${test_name} PRIVATE input_host)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

foreach(test_name
        test_ap_config
        test_ap_settings_merge
        test_toggle_admission
        test_ap_lifecycle)
    add_executable(${test_name} "${test_name}.cpp")
    target_link_libraries(${test_name} PRIVATE wifi_host)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

foreach(test_name
        test_state_json
        test_server_lifecycle
        test_server_controller)
    add_executable(${test_name} "${test_name}.cpp")
    target_link_libraries(${test_name} PRIVATE web_host)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

foreach(test_name
        test_state_health
        test_joy_mapping
        test_arbiter
        test_intent_inbox
        test_arming
        test_stop_protocol
        test_rate_gate
        test_input_grace
        test_joy_observation
        test_rate_starvation
        test_axis_command_ack)
    add_executable(${test_name} "${test_name}.cpp")
    target_link_libraries(${test_name} PRIVATE control_host)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
