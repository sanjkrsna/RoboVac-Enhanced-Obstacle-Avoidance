import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                'launch',
                'online_async_launch.py'
            )
        ]),
        launch_arguments={'slam_params': '/home/ros2/ros2_ws/src/my_bot/config/mapper_params_online_async.yaml'}.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                'launch',
                'navigation_launch.py'
            )
        ])
    )
    

    path_coverage = Node(
        package='path_coverage',
        executable='path_coverage_node.py',
        name='path_coverage',
        output='screen',
        parameters=[{
            "boustrophedon_decomposition": True,
            "border_drive": False,
            "robot_width": 0.3,
            "costmap_max_non_lethal": 15,
            "base_frame": "base_link",
            "global_frame": "map",
            "min_wp_dist": 4.5,
            "num_points": 2
        }]
    )

    image_process = Node(
        package='image_process',
        executable='image_process',
        name='image_process',
        output='screen'
    )

    goal_publisher = Node(
        package='goal_publisher',
        executable='goal_publisher',
        name='goal_publisher',
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        slam,
        nav2,
        path_coverage,
        image_process,
        goal_publisher,
    ])