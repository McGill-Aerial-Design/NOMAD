# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
# Git supplies release provenance; a development build never impersonates a tag.
execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
    OUTPUT_VARIABLE NOMAD_SOURCE_SHA OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
execute_process(COMMAND git ls-tree HEAD third_party/MAVSDK WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
    OUTPUT_VARIABLE mavsdk_tree OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
string(REGEX MATCH "[0-9a-f]+\t" mavsdk_match "${mavsdk_tree}")
string(STRIP "${mavsdk_match}" NOMAD_MAVSDK_SHA)
execute_process(COMMAND git describe --tags --exact-match HEAD WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.."
    OUTPUT_VARIABLE exact_tag OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if("$ENV{GITHUB_REF_TYPE}" STREQUAL "tag")
    set(exact_tag "$ENV{GITHUB_REF_NAME}")
    execute_process(COMMAND git rev-parse "${exact_tag}^{commit}"
        WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.." OUTPUT_VARIABLE tagged_source
        OUTPUT_STRIP_TRAILING_WHITESPACE COMMAND_ERROR_IS_FATAL ANY)
    if(NOT tagged_source STREQUAL NOMAD_SOURCE_SHA)
        message(FATAL_ERROR "Requested release tag does not identify the checked-out source")
    endif()
endif()
if(DEFINED ENV{GITHUB_REF_TYPE} AND NOT "$ENV{GITHUB_REF_TYPE}" STREQUAL "tag")
    set(exact_tag "")
endif()
set(NOMAD_PROJECT_VERSION "0.0.0")
set(NOMAD_RELEASE_VERSION "dev-${NOMAD_SOURCE_SHA}")
set(NOMAD_COMPONENT_VERSION "0.0.0-dev.${NOMAD_SOURCE_SHA}")
set(NOMAD_OFFICIAL false)
execute_process(COMMAND git status --porcelain --untracked-files=normal
    WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/.." OUTPUT_VARIABLE source_changes)
set(NOMAD_SOURCE_DIRTY false)
if(NOT source_changes STREQUAL "")
    set(NOMAD_SOURCE_DIRTY true)
endif()
if(exact_tag MATCHES "^v(0|[1-9][0-9]*)\\.(0|[1-9][0-9]*)\\.(0|[1-9][0-9]*)$")
    if(source_changes STREQUAL "")
        string(SUBSTRING "${exact_tag}" 1 -1 NOMAD_PROJECT_VERSION)
        set(NOMAD_RELEASE_VERSION "${exact_tag}")
        set(NOMAD_COMPONENT_VERSION "${NOMAD_PROJECT_VERSION}")
        set(NOMAD_OFFICIAL true)
    endif()
endif()
