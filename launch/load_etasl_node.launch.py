import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

from launch.actions import DeclareLaunchArgument
# from launch.actions import SetEnvironmentVariable

def generate_launch_description():

    ld = LaunchDescription()

    # urdf_file = "/home/santiregui/ros2_ws/src/etasl_ros2/robot_description/urdf/one_dof_robot.urdf.xml"  # Replace with your URDF file path

    # urdf_file_name = 'robot_description/urdf/one_dof_robot.urdf.xml'
    urdf_file_name = 'urdf/one_dof_robot.urdf.xml'
    package_dir = get_package_share_directory('etasl_ros2')
    print(package_dir)

    urdf_file = os.path.join( get_package_share_directory('etasl_ros2'), urdf_file_name)

    # task_specification_file = os.path.join(package_dir, "etasl/example_ur10.lua")
    # task_specification_file = os.path.join(package_dir, "etasl/moving_jointspace_trap.lua")
    task_specification_file = os.path.join(package_dir, "etasl/move_cartesianspace.lua")

    etasl_node = LifecycleNode(
            package='etasl_ros2',
            executable='etasl_node',
            name='etasl_node',
            namespace = '', #if declared the node becomes visible /namespace/name (e.g. in lifecyle commands from terminal)
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],
            # arguments=['--ros-args', '--log-level', 'info'],
            parameters=[
                {'task_specification_file': task_specification_file},
                {'jointnames': ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]},
                # {'jointnames': ["joint1","joint2"]},
            ],
        )

    ld.add_action(etasl_node)

    return ld