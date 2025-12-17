from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():

    pkg_share = FindPackageShare('joint_mover')

    # 1. Chemin vers le fichier XACRO
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'ppnc_robot.urdf.xacro'])

    # 2. Argument pour le fichier Rviz
    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'rviz', 'ppnc_robot.rviz']),
        description='Absolute path to rviz config file'
    )
    rviz_config = LaunchConfiguration('rviz_config')

    # 3. Traitement XACRO et publication de la description du robot
    robot_description_content = Command(['xacro ', xacro_file])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen'
    )

    # 4. Nœud de mouvement des joints
    joint_mover = Node(
        package='joint_mover',
        executable='move_joints',
        name='joint_mover',
        output='screen'
    )

    # 5. Nœud RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        rviz_arg, 
        robot_state_publisher_node, 
        joint_mover,
        rviz
    ])