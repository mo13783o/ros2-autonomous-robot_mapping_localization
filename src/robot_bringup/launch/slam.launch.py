from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # packages
    bringup_pkg = get_package_share_directory('robot_bringup')

    # include odom
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'full_system.launch.py')
        )
    )

    # include lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'lidar.launch.py')
        )
    )

    # 🔥 Point to your new config file
    slam_params_file = os.path.join(bringup_pkg, 'config', 'slam_params.yaml')

    # SLAM Toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        # 🔥 Pass the parameters file as a launch argument
        launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    return LaunchDescription([
        odom_launch,
        lidar_launch,
        slam
    ])