# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 The NOMAD Authors
"""Keep the shipped ROS node on its restricted telemetry-only dependency."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_ros_node_contains_no_direct_vehicle_commands() -> None:
    node_source = (ROOT / "ros2/nomad_ros/src/node.cpp").read_text(encoding="utf-8")

    for forbidden in (
        "nomad::vehicle::Vehicle",
        "nomad/mavlink/connection.hpp",
        "mavsdk_transport.hpp",
        "make_mavsdk_connection",
        "send_command",
        "send_velocity",
        "set_velocity",
        "goto_location_relative",
        "create_service",
        "/nomad/cmd_vel",
    ):
        assert forbidden not in node_source, f"ROS node regained direct control code: {forbidden}"


def test_ros_node_links_only_the_observation_target() -> None:
    ros_cmake = (ROOT / "ros2/nomad_ros/CMakeLists.txt").read_text(encoding="utf-8")
    root_cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../.. nomad_core_build EXCLUDE_FROM_ALL)" in ros_cmake
    node_link = ros_cmake.split("target_link_libraries(nomad_vehicle_node", maxsplit=1)[1].split(")", maxsplit=1)[0]
    assert "nomad_mavlink_observation" in node_link
    assert "nomad_core" not in node_link
    assert "nomad_mavsdk_connection" not in node_link

    observer_block = root_cmake.split("add_library(nomad_mavlink_observation", maxsplit=1)[1].split(
        "# Command-capable MAVLink transport", maxsplit=1
    )[0]
    assert "src/mavlink/mavlink_observation.cpp" in observer_block
    assert "mavsdk_connection.cpp" not in observer_block
    assert "mavsdk_mavlink_connection.cpp" not in observer_block
    assert (
        "target_link_libraries(nomad_mavlink_observation PUBLIC nomad_mavsdk_validation PRIVATE Threads::Threads)"
        in observer_block
    )
    assert "nomad_mavsdk_connection" not in observer_block


def test_observer_api_is_receive_only() -> None:
    observer_source = (ROOT / "src/mavlink/mavlink_observation.cpp").read_text(encoding="utf-8")

    assert "recvfrom(" in observer_source
    assert "sendto(" not in observer_source
    assert "MavlinkPassthrough" not in observer_source
    assert "mavsdk::Mavsdk" not in observer_source


def test_ros_launch_uses_observation_endpoints_only() -> None:
    launch_service = (ROOT / "scripts/services/nomad_ros_vehicle.sh").read_text(encoding="utf-8")
    environment_example = (ROOT / "config/nomad.env.example").read_text(encoding="utf-8")

    for setting in (
        "NOMAD_ROS_OBSERVATION_PORT",
        "NOMAD_ROS_EXPECTED_SYSTEM_ID",
        "NOMAD_ROS_PUBLISH_RATE_HZ",
    ):
        assert setting in launch_service
        assert setting in environment_example
    assert "NOMAD_MAVLINK_ENDPOINT" not in launch_service
