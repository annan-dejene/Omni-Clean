from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    desc_pkg = get_package_share_directory('omni_clean_description')
    gz_pkg   = get_package_share_directory('omni_clean_gazebo')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'drone.urdf.xacro')
    world_file = os.path.join(gz_pkg, 'worlds', 'facade_world.sdf')

    robot_desc = Command(['xacro ', xacro_file])

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{'robot_description': robot_desc}])

    # Gazebo sim (ros_gz)
    gz = Node(package='ros_gz_sim', executable='gz_sim', output='screen',
              arguments=['-r', world_file])

    # Spawn entity from URDF
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=['-string', robot_desc,
                            '-name', 'omni_clean_drone',
                            '-x', '0', '-y', '0', '-z', '2.0'])

    return LaunchDescription([rsp, gz, spawn])
