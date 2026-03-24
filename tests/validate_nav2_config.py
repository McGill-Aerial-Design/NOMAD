#!/usr/bin/env python3
"""
Validate Nav2 configuration for NOMAD drone.

This script checks:
1. YAML syntax validity
2. Required parameter sections exist
3. Frame ID consistency across local/global costmaps and bt_navigator
4. Essential parameters (BT XML filename, planner_frequency, etc.)
5. Topic references consistency (nvblox_map_slice_topic)

Usage:
    python3 validate_nav2_config.py [config_file]
    python3 validate_nav2_config.py ../config/nav2_drone.yaml
    python3 -m pytest validate_nav2_config.py -v
"""

import sys
import os
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import yaml


class Nav2ConfigValidator:
    """Validator for Nav2 drone YAML configuration."""

    def __init__(self, config_file: str):
        """Initialize validator with config file path."""
        self.config_file = Path(config_file)
        self.config = None
        self.errors: List[str] = []
        self.warnings: List[str] = []

    def validate(self) -> bool:
        """Run full validation suite. Returns True if no errors."""
        if not self._load_yaml():
            return False

        self._check_required_sections()
        self._check_frame_id_consistency()
        self._check_required_parameters()
        self._check_topic_consistency()
        self._check_costmap_configuration()

        return len(self.errors) == 0

    def _load_yaml(self) -> bool:
        """Load and parse YAML file. Returns True if successful."""
        if not self.config_file.exists():
            self.errors.append(f"Config file not found: {self.config_file}")
            return False

        try:
            with open(self.config_file, 'r') as f:
                self.config = yaml.safe_load(f)
            if self.config is None:
                self.errors.append("Config file is empty or contains no valid YAML")
                return False
            return True
        except yaml.YAMLError as e:
            self.errors.append(f"YAML parse error: {e}")
            return False
        except Exception as e:
            self.errors.append(f"Failed to read config file: {e}")
            return False

    def _check_required_sections(self):
        """Verify all required top-level sections exist."""
        required_sections = [
            'bt_navigator',
            'controller_server',
            'planner_server',
            'smoother_server',
            'behavior_server',
            'velocity_smoother',
            'local_costmap',
            'global_costmap',
        ]

        for section in required_sections:
            if section not in self.config:
                self.errors.append(f"Missing required section: {section}")
            else:
                # Check for ros__parameters subsection
                if section.endswith('_server'):
                    if section not in self.config or 'ros__parameters' not in self.config.get(section, {}):
                        self.errors.append(f"Section '{section}' missing 'ros__parameters' subsection")

    def _check_frame_id_consistency(self):
        """Verify frame IDs are consistent across config sections."""
        frame_ids = {}

        # Extract global_frame and robot_base_frame from each section
        sections_to_check = [
            ('bt_navigator', 'bt_navigator'),
            ('controller_server', 'controller_server'),
            ('local_costmap', 'local_costmap.local_costmap'),
            ('global_costmap', 'global_costmap.global_costmap'),
            ('behavior_server', 'behavior_server'),
        ]

        for section_name, dict_path in sections_to_check:
            try:
                parts = dict_path.split('.')
                section = self.config
                for part in parts:
                    section = section.get(part, {})
                    if not isinstance(section, dict):
                        break

                if isinstance(section, dict):
                    params = section.get('ros__parameters', {})
                    if 'global_frame' in params:
                        frame_ids.setdefault('global_frame', []).append(
                            (section_name, params['global_frame'])
                        )
                    if 'robot_base_frame' in params:
                        frame_ids.setdefault('robot_base_frame', []).append(
                            (section_name, params['robot_base_frame'])
                        )
            except (KeyError, AttributeError, TypeError):
                pass

        # Check consistency
        for frame_type, occurrences in frame_ids.items():
            if len(occurrences) > 1:
                values = set(val for _, val in occurrences)
                if len(values) > 1:
                    sections_str = ', '.join(f"{sec}={val}" for sec, val in occurrences)
                    self.warnings.append(
                        f"Inconsistent {frame_type} across sections: {sections_str}"
                    )

    def _check_required_parameters(self):
        """Verify critical parameters are set."""
        checks = [
            ('bt_navigator.ros__parameters.default_bt_xml_filename',
             'BT XML filename for nav2_bringup (required for navigate_to_pose actions)'),
            ('bt_navigator.ros__parameters.global_frame',
             'Global frame for BT navigator'),
            ('bt_navigator.ros__parameters.robot_base_frame',
             'Robot base frame for BT navigator'),
            ('planner_server.ros__parameters.expected_planner_frequency',
             'Planner frequency in Hz'),
            ('controller_server.ros__parameters.controller_frequency',
             'Controller frequency in Hz'),
        ]

        for param_path, description in checks:
            if not self._get_nested_value(param_path):
                self.errors.append(f"Missing required parameter: {param_path} ({description})")

    def _check_topic_consistency(self):
        """Verify nvblox topic references are consistent."""
        nvblox_topics = {}

        # Find all nvblox_map_slice_topic references
        sections = [
            ('local_costmap', self.config.get('local_costmap', {})),
            ('global_costmap', self.config.get('global_costmap', {})),
        ]

        for section_name, section_data in sections:
            if isinstance(section_data, dict):
                costmap_section = section_data.get(
                    section_name.replace('local_', 'local_').replace('global_', 'global_'), {}
                )
                plugins = costmap_section.get('ros__parameters', {}).get('plugins', [])
                for plugin in plugins:
                    if plugin == 'nvblox_layer':
                        nvblox_config = (
                            costmap_section.get('ros__parameters', {})
                            .get('nvblox_layer', {})
                        )
                        topic = nvblox_config.get('nvblox_map_slice_topic')
                        if topic:
                            nvblox_topics[f"{section_name}.nvblox_layer"] = topic

        # Check consistency
        if len(nvblox_topics) > 1:
            unique_topics = set(nvblox_topics.values())
            if len(unique_topics) > 1:
                topics_str = '; '.join(f"{k}={v}" for k, v in nvblox_topics.items())
                self.warnings.append(
                    f"Inconsistent nvblox topics across costmaps: {topics_str}"
                )
            else:
                # All consistent
                pass

    def _check_costmap_configuration(self):
        """Verify costmap configurations are valid."""
        for costmap_type in ['local_costmap', 'global_costmap']:
            costmap_cfg = self.config.get(costmap_type, {})
            costmap_name = costmap_cfg.get(costmap_type, {})
            params = costmap_name.get('ros__parameters', {})

            # Check for required costmap parameters
            required = ['global_frame', 'robot_base_frame', 'width', 'height', 'resolution']
            for param in required:
                if param not in params:
                    self.warnings.append(
                        f"{costmap_type}: Missing recommended parameter '{param}'"
                    )

            # Check plugins exist
            plugins = params.get('plugins', [])
            if not plugins:
                self.errors.append(f"{costmap_type}: No plugins configured")
            else:
                # Check for nvblox_layer
                has_nvblox = False
                for plugin in plugins:
                    if plugin == 'nvblox_layer':
                        has_nvblox = True
                        # Verify nvblox_layer is properly configured
                        nvblox_cfg = params.get('nvblox_layer', {})
                        if 'nvblox_map_slice_topic' not in nvblox_cfg:
                            self.errors.append(
                                f"{costmap_type}.nvblox_layer: Missing nvblox_map_slice_topic"
                            )
                if not has_nvblox:
                    self.warnings.append(f"{costmap_type}: No nvblox_layer plugin detected")

    def _get_nested_value(self, path: str) -> Optional[str]:
        """Get nested value from config using dot-separated path."""
        parts = path.split('.')
        value = self.config
        for part in parts:
            if isinstance(value, dict):
                value = value.get(part)
            else:
                return None
        return value

    def report(self) -> str:
        """Generate validation report."""
        lines = [
            f"Nav2 Configuration Validation: {self.config_file}",
            "=" * 60,
        ]

        if self.errors:
            lines.append("\nErrors:")
            for error in self.errors:
                lines.append(f"  [ERROR] {error}")

        if self.warnings:
            lines.append("\nWarnings:")
            for warning in self.warnings:
                lines.append(f"  [WARN] {warning}")

        if not self.errors and not self.warnings:
            lines.append("\nValidation PASSED - No errors or warnings detected")

        summary_line = f"\nSummary: {len(self.errors)} error(s), {len(self.warnings)} warning(s)"
        lines.append(summary_line)

        return '\n'.join(lines)


def main():
    """Main entry point."""
    # Default to config/nav2_drone.yaml relative to script location
    script_dir = Path(__file__).parent
    default_config = script_dir.parent / 'config' / 'nav2_drone.yaml'

    config_file = sys.argv[1] if len(sys.argv) > 1 else str(default_config)

    validator = Nav2ConfigValidator(config_file)
    success = validator.validate()

    print(validator.report())

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
