from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = '/home/adarsh/amr_ws/src/amr_description/urdf/my_robot.urdf'
    rviz_config_path = '/home/adarsh/amr_ws/src/amr_description/rviz/amr_config.rviz'

    return LaunchDescription([
        # Robot State Publisher (reads URDF and publishes TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}],
            output='screen'
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # RViz with your saved config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])

