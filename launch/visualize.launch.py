import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _strip_root_inertia(urdf_content, root_link_name):
    try:
        robot = ET.fromstring(urdf_content)
    except ET.ParseError:
        return urdf_content

    for link in robot.findall("link"):
        if link.get("name") == root_link_name:
            inertial = link.find("inertial")
            if inertial is not None:
                link.remove(inertial)
            break

    return ET.tostring(robot, encoding="unicode")


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

    ld.add_action(
        DeclareLaunchArgument(
            name="rviz_config",
            default_value=os.path.join(
                get_package_share_directory("igris_c_description_public"),
                "rviz",
                "urdf.rviz",
            ),
            description="RViz config file",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="joint_states_topic",
            default_value="/igris_c/joint_states",
            description="Incoming JointState topic",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="use_joint_state_remap",
            default_value="true",
            description="Remap joint names to match URDF numbering",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="joint_states_remapped_topic",
            default_value="/igris_c/joint_states_remapped",
            description="Remapped JointState topic",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="fixed_frame",
            default_value="base_link",
            description="Fixed frame to anchor the TF tree for RViz",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="fixed_frame_parent",
            default_value="world",
            description="Parent frame for the fixed frame static transform",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            name="root_link",
            default_value="base_link",
            description="Root link to strip inertia from for KDL compatibility",
        )
    )

    def launch_setup(context, *args, **kwargs):
        urdf_pkg = LaunchConfiguration("urdf_package").perform(context)
        urdf_file = LaunchConfiguration("urdf_path").perform(context)
        rviz_config = LaunchConfiguration("rviz_config").perform(context)
        joint_states_topic = LaunchConfiguration("joint_states_topic").perform(context)
        joint_states_remapped_topic = (
            LaunchConfiguration("joint_states_remapped_topic").perform(context)
        )
        use_joint_state_remap = LaunchConfiguration("use_joint_state_remap").perform(
            context
        )
        fixed_frame = LaunchConfiguration("fixed_frame").perform(context)
        fixed_frame_parent = LaunchConfiguration("fixed_frame_parent").perform(context)
        root_link = LaunchConfiguration("root_link").perform(context)
        urdf_full_path = os.path.join(get_package_share_directory(urdf_pkg), urdf_file)

        with open(urdf_full_path, "r") as f:
            urdf_content = f.read()

        # urdf_content = _strip_root_inertia(urdf_content, root_link)

        remapped_topic = (
            joint_states_remapped_topic if use_joint_state_remap == "true" else joint_states_topic
        )

        return [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="fixed_frame_static_tf",
                output="screen",
                arguments=["0", "0", "0", "0", "0", "0", fixed_frame_parent, fixed_frame],
            ),
            Node(
                package="igris_c_description_public",
                executable="joint_state_remap.py",
                name="joint_state_remap",
                output="screen",
                parameters=[{"robot_description": urdf_content}],
                remappings=[
                    ("joint_states_in", joint_states_topic),
                    ("joint_states_out", joint_states_remapped_topic),
                ],
                condition=IfCondition(LaunchConfiguration("use_joint_state_remap")),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": urdf_content}],
                remappings=[("/joint_states", remapped_topic)],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
            ),
        ]

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
