from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # Node joints_state khởi động trước
    joints_state_node = Node(
        package='pcgrad',
        executable='joints_state',
        name='joints_state',
        output='screen',
    )

    # Node run_environment khởi động sau 3 giây để đảm bảo joints_state đã sẵn sàng
    run_environment_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='pcgrad',
                executable='run_environment',
                name='run_environment',
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        joints_state_node,
        run_environment_node,
    ])
