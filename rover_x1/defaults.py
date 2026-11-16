"""Single source for generated ROS defaults; deployment overrides stay separate."""
from dataclasses import asdict

from config import CFG

# Mode owns these flags; bypass_step_max_m is deprecated and no longer used.
OMITTED_PARAMETERS = frozenset({
    'mission_enabled', 'lane_follow_enabled', 'traffic_sequence_enabled',
    'traffic_light_enabled', 'stop_line_enabled', 'bypass_step_max_m',
})


def default_parameters():
    return {name: value for name, value in asdict(CFG).items()
            if name not in OMITTED_PARAMETERS}


def render_defaults():
    import yaml
    return ('# Generated from config.py by tools/generate_ros_config.py; do not edit.\n'
            '# Put deployment overrides in a separate file passed as params_file.\n'
            + yaml.safe_dump({'/**': {'ros__parameters': default_parameters()}},
                             sort_keys=False, allow_unicode=True))
