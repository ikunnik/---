"""Single entry point for the Rover X1 control runtime."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution(
            [FindPackageShare("rover_x1"), "config", "rover.yaml"])),
        DeclareLaunchArgument("mode", default_value="mission",
                              choices=["mission", "avoidance", "traffic", "lane"]),
        DeclareLaunchArgument("check_config", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("dry_run", default_value="false", choices=["true", "false"],
                              description="Simulate motion only; camera and voice still use configured live inputs."),
    ]
    controller = Node(
        package="rover_x1", executable="controller", name="rover_x1_controller",
        output="screen", emulate_tty=True, respawn=False,
        sigterm_timeout="30", sigkill_timeout="10",
        parameters=[LaunchConfiguration("params_file"), {
            "mode": ParameterValue(LaunchConfiguration("mode"), value_type=str),
            "check_config": ParameterValue(LaunchConfiguration("check_config"), value_type=bool),
            "dry_run": ParameterValue(LaunchConfiguration("dry_run"), value_type=bool),
        }],
    )
    def on_exit(event, context):
        if event.returncode != 0 and not context.is_shutdown:
            raise RuntimeError(f"Rover X1 controller failed with exit code {event.returncode}")
        return [EmitEvent(event=Shutdown(reason="Rover X1 controller exited"))]
    finished = RegisterEventHandler(OnProcessExit(target_action=controller, on_exit=on_exit))
    return LaunchDescription(declarations + [finished, controller])
