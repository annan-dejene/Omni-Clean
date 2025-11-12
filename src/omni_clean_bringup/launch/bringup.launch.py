from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    desc_pkg = get_package_share_directory('omni_clean_description')
    gz_pkg   = get_package_share_directory('omni_clean_gazebo')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'drone.urdf.xacro')
    world_file = os.path.join(gz_pkg, 'worlds', 'facade_world.sdf')

    robot_desc = Command(['xacro ', xacro_file])

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # Start Gazebo with the render engine you confirmed works
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '--render-engine', 'ogre', '-r', world_file],
        output='screen'
    )

    # Spawn after a short delay so Gazebo is ready
    spawn = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-string', robot_desc,
                       '-name', 'omni_clean_drone',
                       '-x', '0', '-y', '0', '-z', '2.0'],
            output='screen'
        )]
    )

    return LaunchDescription([rsp, gz, spawn])
