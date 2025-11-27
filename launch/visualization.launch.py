from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for visualization nodes."""

    robot_node = Node(
        package='stihl_visualization',
        executable='polygon_visualization',
        name='polygon_visualization',
        output='screen',
    )

    heatmap_node = Node(
        package='stihl_visualization',
        executable='heatmap_plotter',
        name='heatmap_plotter',
        output='screen',
    )

    return LaunchDescription([
        robot_node,
        heatmap_node,
    ])