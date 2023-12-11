from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction

def generate_launch_description():

    node_1= Node(
        package='wheel_velocities',
        executable='wheel_velocities_publisher',
        output='screen')

    node_2 =  Node(
            package='kinematic_model',
            executable='kinematic_model_node',
            output='screen')
    return LaunchDescription([

        node_1,
        TimerAction(
                period=2.0,
                actions=[node_2],
            )

    ])