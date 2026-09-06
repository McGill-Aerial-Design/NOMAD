cmake_minimum_required(VERSION 3.20)

project(nomad VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
include(CheckCXXCompilerFlag)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(nomad_core
    src/mavlink/fence.cpp
    src/mavlink/params.cpp
    src/mavlink/protocol.cpp
    src/mavlink/udp_commands.cpp
    src/mavlink/udp_connection.cpp
    src/mission/executor.cpp
    src/safety/fence_config.cpp
    src/safety/geofence.cpp
    src/safety/payload.cpp
    src/safety/velocity.cpp
    src/safety/vio_source.cpp
    src/safety/watchdog.cpp
    src/vehicle/output.cpp
    src/vehicle/vehicle.cpp
)

# MAVLink dialect tables come from the pinned third_party/ardupilot-mavlink
# submodule (the exact commit the ArduPilot firmware compiles against), not
# from hand-maintained constants. scripts/dev/generate_mavlink.py runs the
# submodule's own mavgen into the build directory; the generated headers are
# gitignored and never committed.
set(MAVLINK_SUBMODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ardupilot-mavlink")
set(MAVLINK_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated/mavlink")
if(EXISTS "${MAVLINK_SUBMODULE_DIR}/pymavlink/tools/mavgen.py")
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_custom_command(
        OUTPUT "${MAVLINK_GENERATED_DIR}/provenance.txt"
        COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/scripts/dev/generate_mavlink.py"
                --output "${MAVLINK_GENERATED_DIR}"
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/dev/generate_mavlink.py"
                "${MAVLINK_SUBMODULE_DIR}/message_definitions/v1.0/ardupilotmega.xml"
        COMMENT "Generating MAVLink C headers from third_party/ardupilot-mavlink"
        VERBATIM)
    add_custom_target(nomad_mavlink_generate DEPENDS "${MAVLINK_GENERATED_DIR}/provenance.txt")
    add_dependencies(nomad_core nomad_mavlink_generate)
    target_include_directories(nomad_core PRIVATE "${MAVLINK_GENERATED_DIR}")
else()
    message(FATAL_ERROR "MAVLink submodule is missing. Run: git submodule update --init --recursive")
endif()

target_include_directories(nomad_core PUBLIC include)
target_compile_features(nomad_core PUBLIC cxx_std_20)

if(MSVC)
    target_compile_options(nomad_core PRIVATE /W4 /permissive-)
else()
    target_compile_options(nomad_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(WIN32)
    target_link_libraries(nomad_core PRIVATE ws2_32)
endif()

add_executable(nomad src/main.cpp src/cli_commands.cpp)
target_link_libraries(nomad PRIVATE nomad_core)

# The adapter packages embed this file as a subproject; only build the CTest
# targets when the tests/ source tree is actually present.
if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    add_executable(nomad_core_tests tests/core_test.cpp)
    target_link_libraries(nomad_core_tests PRIVATE nomad_core)
    target_compile_definitions(nomad_core_tests PRIVATE NOMAD_TESTS)
    add_test(NAME nomad_core_tests COMMAND nomad_core_tests)

    add_executable(nomad_codec_golden_tests tests/codec_golden_test.cpp)
    target_link_libraries(nomad_codec_golden_tests PRIVATE nomad_core)
    target_compile_definitions(nomad_codec_golden_tests PRIVATE NOMAD_TESTS)
    add_test(NAME nomad_codec_golden_tests COMMAND nomad_codec_golden_tests)

    add_executable(nomad_safety_tests tests/safety_test.cpp)
    target_link_libraries(nomad_safety_tests PRIVATE nomad_core)
    target_compile_definitions(nomad_safety_tests PRIVATE NOMAD_TESTS)
    add_test(NAME nomad_safety_tests COMMAND nomad_safety_tests)

    add_executable(nomad_output_tests tests/output_command_test.cpp)
    target_link_libraries(nomad_output_tests PRIVATE nomad_core)
    target_compile_definitions(nomad_output_tests PRIVATE NOMAD_TESTS)
    add_test(NAME nomad_output_tests COMMAND nomad_output_tests)

    add_executable(nomad_udp_tests tests/udp_connection_test.cpp)
    target_link_libraries(nomad_udp_tests PRIVATE nomad_core)
    target_compile_definitions(nomad_udp_tests PRIVATE NOMAD_TESTS)
    add_test(NAME nomad_udp_tests COMMAND nomad_udp_tests)

    add_executable(nomad_vio_tests tests/vio_source_test.cpp)
    target_link_libraries(nomad_vio_tests PRIVATE nomad_core)
    target_compile_definitions(nomad_vio_tests PRIVATE NOMAD_TESTS)
    add_test(NAME nomad_vio_tests COMMAND nomad_vio_tests)

    add_executable(nomad_fence_config_tests tests/fence_config_test.cpp)
    target_link_libraries(nomad_fence_config_tests PRIVATE nomad_core)
    target_compile_definitions(nomad_fence_config_tests PRIVATE NOMAD_TESTS)
    add_test(NAME nomad_fence_config_tests COMMAND nomad_fence_config_tests)
endif()
