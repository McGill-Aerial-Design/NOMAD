cmake_minimum_required(VERSION 3.22.1)

project(nomad VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# MAVSDK is the transport, not an option: the hand-written codec was removed at
# the Phase E cutover, so a build without MAVSDK would have no way to reach a
# vehicle and the removed path could survive as a hidden fallback. There is
# therefore no enable/disable flag; a missing checkout is a configuration error.
set(NOMAD_MAVSDK_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/MAVSDK" CACHE PATH
    "MAVSDK checkout that provides the transport")

# Keep the MAVSDK build isolated from NOMAD's own options. The child project has
# its own options; normal variables plus CMP0077 prevent those options from
# changing NOMAD's top-level cache or test configuration.
function(add_nomad_mavsdk)
    cmake_policy(SET CMP0077 NEW)
    set(BUILD_TESTING OFF)
    set(BUILD_MAVSDK_SERVER OFF)
    set(BUILD_WITHOUT_CURL ON)
    set(BUILD_SHARED_LIBS OFF)
    set(SUPERBUILD ON)
    # mavlink_passthrough is not an ENABLED_PLUGINS option: MAVSDK's plugin
    # CMakeLists always adds it, so it is available without listing it here.
    # Action owns relative goto; Offboard owns one-shot velocity; Geofence and Param own their transports.
    set(ENABLED_PLUGINS telemetry geofence param action offboard)
    set(MAVLINK_DIALECT ardupilotmega)
    add_subdirectory("${NOMAD_MAVSDK_SOURCE_DIR}/cpp" "${CMAKE_BINARY_DIR}/mavsdk")
    # Record the effective child options for resource evidence without exporting cache defaults.
    string(JOIN "\",\"" resource_plugins ${ENABLED_PLUGINS} mavlink_passthrough)
    file(WRITE "${CMAKE_BINARY_DIR}/resource-build-config.json"
        "{\"shared\":false,\"server\":false,\"curl\":false,\"superbuild\":true,\"mavsdk_build_testing\":false,"
        "\"dialect\":\"${MAVLINK_DIALECT}\",\"plugins\":[\"${resource_plugins}\"]}\n")
endfunction()

if(NOT EXISTS "${NOMAD_MAVSDK_SOURCE_DIR}/cpp/CMakeLists.txt")
    message(FATAL_ERROR
        "MAVSDK source is missing. Run: git submodule update --init --recursive")
endif()
add_nomad_mavsdk()

set(NOMAD_MAVSDK_INSTALL_DIR "${CMAKE_BINARY_DIR}/mavsdk/third_party/install")

add_library(nomad_mavsdk_validation src/mavlink/mavsdk_validation.cpp)
target_include_directories(nomad_mavsdk_validation PUBLIC include)
target_compile_features(nomad_mavsdk_validation PUBLIC cxx_std_20)
if(MSVC)
    target_compile_options(nomad_mavsdk_validation PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(nomad_mavsdk_validation PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_library(nomad_core
    src/mission/executor.cpp
    src/safety/fence_config.cpp
    src/safety/geofence.cpp
    src/safety/payload.cpp
    src/safety/velocity.cpp
    src/safety/velocity_config.cpp
    src/safety/vio_source.cpp
    src/safety/watchdog.cpp
    src/telemetry/identity.cpp
    src/util/parse.cpp
    src/vehicle/operation.cpp
    src/vehicle/output.cpp
    src/vehicle/vehicle.cpp
    src/vehicle/vehicle_vtol_takeoff.cpp
    src/vehicle/vehicle_fence.cpp
    src/vehicle/vehicle_payload.cpp
    src/vehicle/vehicle_quadplane_landing.cpp
      src/vehicle/vehicle_route.cpp
      src/vehicle/vehicle_recovery.cpp
    src/vehicle/vehicle_transition.cpp
    src/vehicle/vehicle_transition_to_vtol.cpp
    src/vehicle/vehicle_velocity.cpp
)

# MAVLink packing lives in the MAVSDK transport: the transport builds frames with
# MAVSDK's own generated codec, so nomad_core has no dialect headers of its own
# and no build-time mavgen step. The pinned third_party/ardupilot-mavlink
# submodule stays because the Python command-id gate resolves hand-typed ids
# against its definition.

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

add_executable(nomad
    src/main.cpp
    src/runtime/cli_client.cpp
    src/cli_arguments.cpp
    src/cli_command_table.cpp)
# The CLI uses the pinned JSON headers but does not link the MAVSDK vehicle transport.
target_include_directories(nomad PRIVATE "${NOMAD_MAVSDK_INSTALL_DIR}/include")
add_dependencies(nomad mavsdk)
target_link_libraries(nomad PRIVATE nomad_core)

add_library(nomad_local_crypto src/runtime/auth_proof.cpp)
target_compile_features(nomad_local_crypto PUBLIC cxx_std_20)
if(WIN32)
    target_link_libraries(nomad_local_crypto PRIVATE bcrypt)
else()
    find_package(OpenSSL REQUIRED COMPONENTS Crypto)
    target_link_libraries(nomad_local_crypto PRIVATE OpenSSL::Crypto)
endif()
target_link_libraries(nomad PRIVATE nomad_local_crypto)

add_library(nomad_runtime_core
    src/runtime/client_auth.cpp
    src/runtime/protected_file.cpp
    src/runtime/audit_journal.cpp
    src/runtime/ipc_server.cpp
    src/runtime/runtime.cpp)
target_include_directories(nomad_runtime_core PUBLIC include PRIVATE
    "${NOMAD_MAVSDK_INSTALL_DIR}/include")
target_compile_features(nomad_runtime_core PUBLIC cxx_std_20)
add_dependencies(nomad_runtime_core mavsdk)
target_link_libraries(nomad_runtime_core PUBLIC nomad_core PRIVATE nomad_local_crypto)
if(WIN32)
    target_link_libraries(nomad_runtime_core PUBLIC ws2_32 advapi32)
endif()
if(MSVC)
    target_compile_options(nomad_runtime_core PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(nomad_runtime_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(PROJECT_IS_TOP_LEVEL)
    add_executable(nomad-runtime tools/runtime/main.cpp tools/runtime/lifecycle.cpp tools/runtime/service_config.cpp)
    target_include_directories(nomad-runtime PRIVATE src/runtime)
    target_include_directories(nomad-runtime PRIVATE "${NOMAD_MAVSDK_INSTALL_DIR}/include")
    target_link_libraries(nomad-runtime PRIVATE nomad_runtime_core nomad_mavsdk_connection)
    target_compile_definitions(nomad-runtime PRIVATE NOMAD_VERSION="${PROJECT_VERSION}")

    # Direct vehicle access exists only for SITL and transport qualification.
    # This target is excluded from the default build and is never installed.
    add_executable(nomad-qualification EXCLUDE_FROM_ALL
        src/qualification/main.cpp
        src/qualification/arguments.cpp
        src/qualification/commands.cpp
        src/qualification/demos.cpp
        src/qualification/velocity.cpp
        src/runtime/cli_client.cpp
        src/cli_arguments.cpp
        src/cli_command_table.cpp)
    target_include_directories(nomad-qualification PRIVATE src "${NOMAD_MAVSDK_INSTALL_DIR}/include")
    target_link_libraries(nomad-qualification PRIVATE nomad_core nomad_mavsdk_connection nomad_local_crypto)
endif()

if(PROJECT_IS_TOP_LEVEL)
    include(GNUInstallDirs)

    # The core release is an executable package. Static MAVSDK internals are linked
    # into the runtime, so the staged binaries do not depend on the build tree. Headers,
    # configuration templates and the reviewed dependency notices travel with the
    # executable for clean install and rollback review.
    install(TARGETS nomad nomad-runtime RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT nomad_runtime)
    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT nomad_runtime)
    install(FILES LICENSE NOTICE DESTINATION ${CMAKE_INSTALL_DATADIR}/nomad COMPONENT nomad_runtime)
    install(FILES config/README.md config/nomad.env.example
            DESTINATION ${CMAKE_INSTALL_DATADIR}/nomad/config COMPONENT nomad_runtime)
    install(FILES infra/runtime/nomad-runtime.service.in infra/runtime/runtime.example.json
            infra/runtime/install_systemd.py infra/runtime/Manage-NomadRuntime.ps1
            DESTINATION ${CMAKE_INSTALL_DATADIR}/nomad/lifecycle COMPONENT nomad_runtime)
    install(FILES docs/operations.md DESTINATION ${CMAKE_INSTALL_DATADIR}/nomad COMPONENT nomad_runtime)
    install(DIRECTORY licenses/mavsdk-phase-a/
            DESTINATION ${CMAKE_INSTALL_DATADIR}/nomad/licenses/mavsdk-phase-a COMPONENT nomad_runtime)

    set(CPACK_PACKAGE_NAME "nomad-core")
    set(CPACK_PACKAGE_VENDOR "The NOMAD Authors")
    set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "NOMAD C++ vehicle core, runtime and CLI")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
    set(CPACK_PACKAGE_DIRECTORY "${CMAKE_BINARY_DIR}")
    set(CPACK_COMPONENTS_ALL nomad_runtime)
    set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)
    set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)
    set(CPACK_GENERATOR "ZIP;TGZ")
    include(CPack)
endif()

# MAVSDK's embedded fmt target is directory-scoped and is not a reliable
# dependency for a parent project. Import the generated archive explicitly so
# static consumers link the same artifact on Linux and Windows.
# fmt installs a per-configuration archive (`fmtd` in Debug), so the location
# has to name both: the Debug CTest tree and the Release transport tree are the
# two configurations the Pixi tasks build.
set(NOMAD_MAVSDK_FMT_LIBRARY
    "${NOMAD_MAVSDK_INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}fmt${CMAKE_STATIC_LIBRARY_SUFFIX}")
set(NOMAD_MAVSDK_FMT_DEBUG_LIBRARY
    "${NOMAD_MAVSDK_INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}fmtd${CMAKE_STATIC_LIBRARY_SUFFIX}")
add_library(nomad_mavsdk_fmt STATIC IMPORTED GLOBAL)
set_target_properties(nomad_mavsdk_fmt PROPERTIES
    IMPORTED_LOCATION "${NOMAD_MAVSDK_FMT_LIBRARY}"
    IMPORTED_LOCATION_DEBUG "${NOMAD_MAVSDK_FMT_DEBUG_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${NOMAD_MAVSDK_INSTALL_DIR}/include")

# Shared MAVSDK discovery helpers used by the command-capable transport.
add_library(nomad_mavsdk_system src/mavlink/mavsdk_system.cpp)
target_include_directories(nomad_mavsdk_system PUBLIC include PRIVATE
    "${NOMAD_MAVSDK_SOURCE_DIR}/cpp/src/mavsdk/core/include"
)
target_compile_features(nomad_mavsdk_system PUBLIC cxx_std_20)
add_dependencies(nomad_mavsdk_system mavsdk)
target_link_libraries(nomad_mavsdk_system PRIVATE mavsdk nomad_mavsdk_fmt)
if(MSVC)
    target_compile_options(nomad_mavsdk_system PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(nomad_mavsdk_system PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Receive-only UDP telemetry observation used by ROS and the connectivity smoke.
# MAVLink parsing uses the generated dialect header but owns no MAVSDK connection.
add_library(nomad_mavlink_observation src/mavlink/mavlink_observation.cpp)
target_include_directories(nomad_mavlink_observation PUBLIC include)
target_include_directories(nomad_mavlink_observation SYSTEM PRIVATE
    "${NOMAD_MAVSDK_INSTALL_DIR}/include")
target_compile_features(nomad_mavlink_observation PUBLIC cxx_std_20)
add_dependencies(nomad_mavlink_observation mavsdk)
target_link_libraries(nomad_mavlink_observation PUBLIC nomad_mavsdk_validation PRIVATE Threads::Threads)
if(WIN32)
    target_link_libraries(nomad_mavlink_observation PRIVATE ws2_32)
endif()
if(MSVC)
    target_compile_options(nomad_mavlink_observation PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(nomad_mavlink_observation PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Command-capable MAVLink transport used only by the runtime, qualification
# driver and connection tests. The installed `nomad` CLI is not a consumer.
add_library(nomad_mavsdk_connection
    src/mavlink/mavsdk_action.cpp
    src/mavlink/mavsdk_connection_state.cpp
    src/mavlink/mavsdk_command_result.cpp
    src/mavlink/mavsdk_fence.cpp
    src/mavlink/mavsdk_mavlink_connection.cpp
    src/mavlink/mavsdk_mavlink_quadplane.cpp
    src/mavlink/mavsdk_route.cpp
    src/mavlink/mavsdk_vtol_state.cpp
    src/mavlink/mavsdk_velocity.cpp)
target_include_directories(nomad_mavsdk_connection PUBLIC include PRIVATE
    "${NOMAD_MAVSDK_SOURCE_DIR}/cpp/src/mavsdk/core/include"
    "${NOMAD_MAVSDK_SOURCE_DIR}/cpp/src/mavsdk/plugins/action/include"
    "${NOMAD_MAVSDK_SOURCE_DIR}/cpp/src/mavsdk/plugins/geofence/include"
    "${NOMAD_MAVSDK_SOURCE_DIR}/cpp/src/mavsdk/plugins/param/include"
    "${NOMAD_MAVSDK_SOURCE_DIR}/cpp/src/mavsdk/plugins/telemetry/include"
    "${NOMAD_MAVSDK_SOURCE_DIR}/cpp/src/mavsdk/plugins/mavlink_passthrough/include"
)
target_compile_features(nomad_mavsdk_connection PUBLIC cxx_std_20)
add_dependencies(nomad_mavsdk_connection mavsdk)
target_link_libraries(
    nomad_mavsdk_connection PUBLIC nomad_mavsdk_validation
    PRIVATE nomad_mavsdk_system mavsdk nomad_mavsdk_fmt)
if(MSVC)
    target_compile_options(nomad_mavsdk_connection PRIVATE /W4 /permissive- /EHsc)
else()
    target_compile_options(nomad_mavsdk_connection PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Non-installed direct connect/status smoke retained for explicit qualification.
add_executable(nomad_mavsdk_connectivity_smoke EXCLUDE_FROM_ALL examples/mavsdk_connectivity_smoke.cpp)
target_include_directories(nomad_mavsdk_connectivity_smoke PRIVATE include)
add_dependencies(nomad_mavsdk_connectivity_smoke mavsdk)
target_link_libraries(nomad_mavsdk_connectivity_smoke PRIVATE nomad_mavlink_observation)

# The adapter packages embed this file as a subproject; only build the CTest
# targets when the tests/ source tree is actually present.
if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/NomadTests.cmake")

    if(PROJECT_IS_TOP_LEVEL)
        add_executable(nomad_runtime_tests tests/runtime_ipc_test.cpp tools/runtime/lifecycle.cpp)
        target_include_directories(nomad_runtime_tests PRIVATE "${NOMAD_MAVSDK_INSTALL_DIR}/include")
        target_link_libraries(nomad_runtime_tests PRIVATE nomad_runtime_core)
        target_compile_definitions(nomad_runtime_tests PRIVATE NOMAD_TESTS)
        if(WIN32)
            target_compile_definitions(nomad_runtime_tests PRIVATE NOMAD_SERVICE_TEST)
        endif()
        add_test(NAME nomad_runtime_tests COMMAND nomad_runtime_tests)
    endif()

    add_executable(nomad_mavsdk_validation_tests tests/mavsdk_validation_test.cpp)
    target_link_libraries(nomad_mavsdk_validation_tests PRIVATE nomad_mavsdk_validation)
    if(MSVC)
        target_compile_options(nomad_mavsdk_validation_tests PRIVATE /W4 /permissive- /EHsc)
    endif()
    add_test(NAME nomad_mavsdk_validation_tests COMMAND nomad_mavsdk_validation_tests)

    nomad_add_core_test(nomad_core_tests tests/core_test.cpp)
    nomad_add_core_test(nomad_fake_connection_contract_tests tests/fake_connection_contract_test.cpp)
    nomad_add_core_test(nomad_vehicle_config_tests tests/vehicle_config_test.cpp)

    add_executable(nomad_vehicle_primitives_tests tests/vehicle_primitives_test.cpp)
    target_include_directories(nomad_vehicle_primitives_tests PRIVATE src/vehicle)
    target_link_libraries(nomad_vehicle_primitives_tests PRIVATE nomad_core)
    target_compile_definitions(nomad_vehicle_primitives_tests PRIVATE NOMAD_TESTS)
    add_test(NAME nomad_vehicle_primitives_tests COMMAND nomad_vehicle_primitives_tests)

    nomad_add_core_test(nomad_quadplane_vtol_takeoff_tests tests/quadplane_vtol_takeoff_test.cpp)
    nomad_add_core_test(nomad_quadplane_transition_tests tests/vehicle/quadplane/quadplane_transition_test.cpp)
    nomad_add_core_test(nomad_quadplane_transition_to_vtol_tests
        tests/vehicle/quadplane/quadplane_transition_to_vtol_test.cpp)
    nomad_add_core_test(nomad_quadplane_vtol_landing_tests tests/vehicle/quadplane/quadplane_vtol_landing_test.cpp)
    nomad_add_core_test(nomad_quadplane_route_tests tests/vehicle/quadplane/quadplane_route_test.cpp)
    nomad_add_core_test(nomad_quadplane_recovery_tests tests/vehicle/quadplane/quadplane_recovery_test.cpp)
    nomad_add_core_test(nomad_safety_tests tests/safety_test.cpp)
    nomad_add_core_test(nomad_output_tests tests/output_command_test.cpp)
    nomad_add_core_test(nomad_vio_tests tests/vio_source_test.cpp)
    nomad_add_core_test(nomad_aircraft_identity_tests tests/aircraft_identity_test.cpp)
    nomad_add_core_test(nomad_operation_capability_tests tests/operation_capability_test.cpp)
    nomad_add_core_test(nomad_velocity_config_tests tests/velocity_config_test.cpp)
    nomad_add_core_test(nomad_fence_config_tests tests/fence_config_test.cpp)

    add_executable(nomad_mavsdk_connection_tests tests/mavsdk_connection_test.cpp)
    target_link_libraries(nomad_mavsdk_connection_tests PRIVATE nomad_mavsdk_connection nomad_core)
    target_compile_definitions(nomad_mavsdk_connection_tests PRIVATE NOMAD_TESTS)
    add_test(NAME nomad_mavsdk_connection_tests COMMAND nomad_mavsdk_connection_tests)

    if(PROJECT_IS_TOP_LEVEL)
        add_executable(nomad_mavsdk_authority_wire_probe EXCLUDE_FROM_ALL tests/mavsdk_authority_wire_probe.cpp)
        target_link_libraries(nomad_mavsdk_authority_wire_probe PRIVATE nomad_mavsdk_connection nomad_core)
    endif()

    # SR-LNK-03. The peer-driven scenarios run from
    # scripts/dev/mavsdk_connection_fixture.py; CTest covers the no-link
    # failure paths, which need no peer.
    add_executable(nomad_mavsdk_zero_delivery_tests tests/mavsdk_zero_delivery_test.cpp)
    target_link_libraries(nomad_mavsdk_zero_delivery_tests PRIVATE nomad_mavsdk_connection nomad_core)
    target_compile_definitions(nomad_mavsdk_zero_delivery_tests PRIVATE NOMAD_TESTS)
    add_test(NAME nomad_mavsdk_zero_delivery_tests COMMAND nomad_mavsdk_zero_delivery_tests)
endif()
