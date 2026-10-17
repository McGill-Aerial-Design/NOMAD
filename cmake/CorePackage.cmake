# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Core runtime installation and reviewed release package registration.
get_filename_component(NOMAD_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(NOMAD_PACKAGE_PLATFORM "")
if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(NOMAD_PACKAGE_PLATFORM windows)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(NOMAD_PACKAGE_PLATFORM linux)
endif()

if(NOT NOMAD_PACKAGE_PLATFORM OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8
   OR NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(AMD64|amd64|x86_64|x64)$")
    set(rejection "Reviewed NOMAD release packages require Windows or Linux x86-64")
    install(CODE "message(FATAL_ERROR \"${rejection}\")" COMPONENT nomad_runtime)
    # Replace earlier generated metadata too, so reconfiguration cannot reuse a Linux label.
    file(WRITE "${CMAKE_BINARY_DIR}/CPackConfig.cmake" "message(FATAL_ERROR \"${rejection}\")\n")
    file(WRITE "${CMAKE_BINARY_DIR}/package-identity.json" "{\"unsupported_release_target\":true}\n")
    message(STATUS "NOMAD development build: release packaging unavailable on this target")
    return()
endif()

include(GNUInstallDirs)
if(NOMAD_FIXTURE_VERSION OR NOMAD_SOURCE_ARCHIVE)
    install(CODE "message(FATAL_ERROR \"Fixture/source-archive runtimes cannot be installed or packaged\")"
            COMPONENT nomad_runtime)
endif()

# The core release is an executable package. Static MAVSDK internals are linked
# into the runtime, so the staged binaries do not depend on the build tree. Headers,
# configuration templates and the reviewed dependency notices travel with the
# executable for clean install and rollback review.
install(TARGETS nomad nomad-runtime RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT nomad_runtime)
install(FILES "${NOMAD_PACKAGE_SOURCE_DIR}/LICENSE" "${NOMAD_PACKAGE_SOURCE_DIR}/NOTICE"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/nomad COMPONENT nomad_runtime)
install(FILES "${NOMAD_PACKAGE_SOURCE_DIR}/config/nomad.env.example"
        "${NOMAD_PACKAGE_SOURCE_DIR}/config/actuators.example.json"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/nomad/config COMPONENT nomad_runtime)
install(FILES "${NOMAD_PACKAGE_SOURCE_DIR}/infra/runtime/nomad-runtime.service.in"
        "${NOMAD_PACKAGE_SOURCE_DIR}/infra/runtime/runtime.example.json"
        "${NOMAD_PACKAGE_SOURCE_DIR}/infra/runtime/install_systemd.py"
        "${NOMAD_PACKAGE_SOURCE_DIR}/infra/runtime/Manage-NomadRuntime.ps1"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/nomad/lifecycle COMPONENT nomad_runtime)
install(FILES "${NOMAD_PACKAGE_SOURCE_DIR}/docs/operations.md"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/nomad COMPONENT nomad_runtime)
install(DIRECTORY "${NOMAD_PACKAGE_SOURCE_DIR}/licenses/mavsdk-phase-a/"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/nomad/licenses/mavsdk-phase-a COMPONENT nomad_runtime)

if(NOMAD_PACKAGE_PLATFORM STREQUAL "windows")
    set(NOMAD_PACKAGE_REQUIRED "\"bin/nomad.exe\", \"bin/nomad-runtime.exe\"")
else()
    set(NOMAD_PACKAGE_REQUIRED "\"bin/nomad\", \"bin/nomad-runtime\"")
endif()
configure_file("${NOMAD_PACKAGE_SOURCE_DIR}/cmake/package-identity.json.in"
               "${CMAKE_BINARY_DIR}/package-identity.json" @ONLY)
install(FILES "${CMAKE_BINARY_DIR}/package-identity.json" DESTINATION . COMPONENT nomad_runtime)
set(CPACK_PACKAGE_FILE_NAME "nomad-core-${NOMAD_RELEASE_VERSION}-${NOMAD_PACKAGE_PLATFORM}-x86_64")
set(CPACK_PACKAGE_NAME "nomad-core")
set(CPACK_PACKAGE_VENDOR "The NOMAD Authors")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "NOMAD C++ vehicle core, runtime and CLI")
set(CPACK_RESOURCE_FILE_LICENSE "${NOMAD_PACKAGE_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CPACK_COMPONENTS_ALL nomad_runtime)
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)
set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)
set(CPACK_GENERATOR "ZIP;TGZ")
include(CPack)
