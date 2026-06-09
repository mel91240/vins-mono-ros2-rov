from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ------------------------------------------------------------------
    # Bringup complet VINS-Mono pour le ROV : feature_tracker + estimator
    # + pose_graph, tous lancés avec les BONS namespaces (/vins_estimator,
    # /pose_graph, /feature_tracker). C'est ce qui permet au pose_graph de
    # recevoir les keyframes -> la fermeture de boucle fonctionne.
    # (Lancer ces noeuds via `ros2 run` sans namespace casse la loop closure.)
    # ------------------------------------------------------------------

    config_pkg_path = get_package_share_directory('config_pkg')

    # Chemin du fichier de calibration/config (mclab par defaut, surchargeable)
    default_config = (
        '/home/melanie/rov_ws/src/VINS-MONO-ROS2/config_pkg/config/'
        'mclab/mclab_1_config.yaml'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Chemin absolu du fichier de config YAML (camera + VINS)')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Utiliser l\'horloge du bag (/clock) -> true en rejeu, false en live')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Lancer RViz avec la config VINS')

    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # config_pkg/ (sert au feature_tracker pour le masque fisheye, etc.)
    vins_folder = PathJoinSubstitution([config_pkg_path, 'config/../'])
    support_path = PathJoinSubstitution([config_pkg_path, 'support_files'])
    rviz_config = PathJoinSubstitution([config_pkg_path, 'config/vins_euroc_rviz.rviz'])

    feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker',
        name='feature_tracker',
        namespace='feature_tracker',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'vins_folder': vins_folder,
            'use_sim_time': use_sim_time,
        }],
    )

    vins_estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator',
        name='vins_estimator',
        namespace='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'vins_folder': vins_folder,
            'use_sim_time': use_sim_time,
        }],
    )

    pose_graph_node = Node(
        package='pose_graph',
        executable='pose_graph',
        name='pose_graph',
        namespace='pose_graph',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'support_file': support_path,
            'use_sim_time': use_sim_time,
            'visualization_shift_x': 0,
            'visualization_shift_y': 0,
            'skip_cnt': 0,
            'skip_dis': 0.0,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        rviz_arg,
        LogInfo(msg=['[rov_bringup] config_file : ', config_file]),
        LogInfo(msg=['[rov_bringup] namespaces : /feature_tracker /vins_estimator /pose_graph']),
        feature_tracker_node,
        vins_estimator_node,
        pose_graph_node,
        rviz_node,
    ])
