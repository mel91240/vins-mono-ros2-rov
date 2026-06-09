from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    config_pkg_path = get_package_share_directory('config_pkg')

    # 1. Déclaration de l'argument pour le fichier de configuration (EuRoC par défaut)
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='/home/melanie/rov_ws/src/VINS-MONO-ROS2/config_pkg/config/euroc/euroc_config.yaml',
        description='Chemin absolu vers le fichier de configuration YAML'
    )

    # Récupération de la configuration choisie
    config_path = LaunchConfiguration('config_file')

    vins_path = PathJoinSubstitution([
        config_pkg_path,
        'config/../'
    ])

    # Déclaration de l'argument pour le temps simulé
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bag) clock if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Define the node
    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker',
        name='feature_tracker',
        namespace='feature_tracker',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path,
            'use_sim_time': use_sim_time
        }]
    )

    rviz_config_path = PathJoinSubstitution([
        config_pkg_path,
        'config/vins_euroc_rviz.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        LogInfo(msg=['[feature tracker launch] Chargement de la config: ', config_path]),
        feature_tracker_node,
        rviz_node
    ])
