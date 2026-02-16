import os
import yaml

from tempfile import NamedTemporaryFile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

this_share_dir = get_package_share_directory("auto_apms_nav2")
bringup_dir = get_package_share_directory("nav2_bringup")
params_file = os.path.join(bringup_dir, "params", "nav2_params.yaml")
bt_navigator_params_file = os.path.join(
    this_share_dir, "config", "bt_navigator_params.yaml"
)


def generate_launch_description():
    with open(params_file, "r") as f:
        params = yaml.safe_load(f)

    with open(bt_navigator_params_file, "r") as f:
        bt_navigator_params = yaml.safe_load(f)

    # Replace bt_navigator key in nav2_params.yaml with the contents of bt_navigator_params.yaml
    # and create a temporary params file
    params["bt_navigator"] = bt_navigator_params["bt_navigator"]
    temp_file = NamedTemporaryFile(mode="w", delete=False, suffix=".yaml")
    yaml.dump(params, temp_file)
    temp_file.close()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="True",
                description="Whether to launch in headless mode (no Gazebo GUI)",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, "launch", "tb3_simulation_launch.py")
                ),
                launch_arguments={
                    "params_file": temp_file.name,
                    "headless": LaunchConfiguration("headless"),
                }.items(),
            ),
        ]
    )
