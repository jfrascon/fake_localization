from launch.substitutions import LaunchConfiguration

from launch import LaunchContext


def to_float(ctx: LaunchContext, name: str) -> float:
    try:
        return float(LaunchConfiguration(name).perform(ctx))
    except ValueError as exc:
        raise ValueError(f'Invalid value for {name}. Must be a float.') from exc
