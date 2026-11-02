"""Robot-profile parameters (joints, gripper, cameras) for the policy clients.

They are declared with a type but no default value, so they must come from the robot profile
(vela_robots/config/<robot>/robot.yaml, loaded by every launch file). A node started without it,
or with a profile missing a key, exits at startup naming the parameter instead of silently
running on another robot's values.
"""

from rclpy.exceptions import ParameterUninitializedException
from rclpy.parameter import Parameter

T = Parameter.Type


def declare_robot_params(node, params: dict) -> None:
    """Declare {name: Parameter.Type} without values."""
    for name, ptype in params.items():
        node.declare_parameter(name, ptype)


def robot_param(node, name: str):
    """Value of a robot-profile parameter; exits naming it if the profile didn't set it."""
    try:
        return node.get_parameter(name).value
    except ParameterUninitializedException:
        raise SystemExit(
            f"Parameter '{name}' is not set. It is a robot fact: set it in "
            "vela_robots/config/<robot>/robot.yaml (the launch files load it with robot:=<name>)."
        ) from None
