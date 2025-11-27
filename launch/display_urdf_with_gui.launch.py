import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="urdf_package",
            default_value="igris_c_description_public",
            description="Package name containing the URDF",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="urdf_path",
            default_value="urdf/igris_c_v2.urdf",
            description="Path to URDF file inside the package",
        )
    )

    def launch_setup(context, *args, **kwargs):
        urdf_pkg = LaunchConfiguration("urdf_package").perform(context)
        urdf_file = LaunchConfiguration("urdf_path").perform(context)
        urdf_full_path = os.path.join(get_package_share_directory(urdf_pkg), urdf_file)

        with open(urdf_full_path, "r") as f:
            urdf_content = f.read()

        return [
            # GUI sliders to control joints
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                remappings=[("/joint_states", "/igris_c/joint_states")],
            ),
            # Publishes TFs using URDF
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": urdf_content}],
                remappings=[("/joint_states", "/igris_c/joint_states")],
            ),
            # Launch RViz
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(get_package_share_directory(urdf_pkg), "rviz", "urdf.rviz"),
                ]
                if os.path.exists(
                    os.path.join(get_package_share_directory(urdf_pkg), "rviz", "urdf.rviz")
                )
                else [],
            ),
        ]

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
