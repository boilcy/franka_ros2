import ast
import os

from collections.abc import Sized

import yaml


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f'File not found: {file_path}')

    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def parse_string_list(string_list_repr):
    try:
        parsed_value = ast.literal_eval(string_list_repr)
    except (ValueError, SyntaxError):
        cleaned = string_list_repr.strip('[]').replace("'", '').replace('"', '')
        return [item.strip() for item in cleaned.split(',') if item.strip()]

    if not isinstance(parsed_value, list):
        raise ValueError(f'Expected a list representation, got: {string_list_repr}')

    return parsed_value


def is_duo_config(config):
    duo_keys = {'robot_types', 'robot_ips', 'arm_prefixes'}
    return duo_keys.issubset(config.keys())


def validate_duo_arrays_length(robot_types_list, robot_ips_list, arm_prefixes_list):
    _assert_same_length(robot_types_list, robot_ips_list, arm_prefixes_list)


def validate_arm_prefixes_unique(arm_prefixes_list):
    duplicates = sorted(
        {prefix for prefix in arm_prefixes_list if arm_prefixes_list.count(prefix) > 1}
    )

    if duplicates:
        raise ValueError(
            'arm_prefixes must be unique. '
            f'arm_prefixes: {arm_prefixes_list}; duplicate values: {duplicates}'
        )


def _assert_same_length(*items: Sized):
    if not items:
        return

    expected_length = len(items[0])
    for item in items[1:]:
        if len(item) != expected_length:
            raise ValueError(
                'Duo configuration arrays must have the same length. '
                f'Lengths: {[len(candidate) for candidate in items]}'
            )
