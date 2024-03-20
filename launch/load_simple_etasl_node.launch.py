import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # urdf_file = "/home/santiregui/ros2_ws/src/etasl_ros2/robot_description/urdf/one_dof_robot.urdf.xml"  # Replace with your URDF file path

    # urdf_file_name = 'robot_description/urdf/one_dof_robot.urdf.xml'
    urdf_file_name = 'urdf/one_dof_robot.urdf.xml'
    package_dir = get_package_share_directory('etasl_ros2')
    print(package_dir)

    urdf_file = os.path.join( get_package_share_directory('etasl_ros2'), urdf_file_name)

    # task_specification_file = os.path.join(package_dir, "etasl/example_ur10.lua")
    task_specification_file = os.path.join(package_dir, "etasl/moving_jointspace_trap.lua")
    # task_specification_file = os.path.join(package_dir, "etasl/move_cartesianspace.lua")
    print("THE NAME IS: " + task_specification_file)


    return LaunchDescription([
        Node(
            package='etasl_ros2',
            executable='simple_etasl_node',
            name='simple_etasl_node',
            output='screen',
            parameters=[
                {'task_specification_file': task_specification_file},
                {'jointnames': ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]},
                # {'jointnames': ["joint1","joint2"]},
            ],
        ),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),
    ])