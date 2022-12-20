import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        launch_ros.actions.Node(
            package='imu_turtle',
            executable='node1',
            name='node1'),
        launch_ros.actions.Node(
            package='imu_turtle',
            executable='node2',
            name='node2'),
        launch_ros.actions.Node(
            package='imu_turtle',
            executable='node3',
            name='node3'),
])
