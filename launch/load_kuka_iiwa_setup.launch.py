import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # urdf_file = "/workspaces/colcon_ws/src/etasl_ros2/robot_description/urdf/one_dof_robot.urdf.xml"  # Replace with your URDF file path

    # urdf_file_name = 'robot_description/urdf/one_dof_robot.urdf.xml'
    urdf_file_name = 'robot_description/urdf/kuka_iiwa/use_case_setup_iiwa.urdf'
    package_dir = get_package_share_directory('etasl_ros2')
    print(package_dir)

    urdf_file = os.path.join( get_package_share_directory('etasl_ros2'), urdf_file_name)
    print("THE NAME IS: " + urdf_file)
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf_file]
        ),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/workspaces/colcon_ws/src/etasl_ros2/robot_description/rviz_config.rviz']  # Replace with your RViz config file path
        )
    ])