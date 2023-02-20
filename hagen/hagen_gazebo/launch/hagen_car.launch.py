import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    navpkg = 'autocar_nav'
    gzpkg = 'autocar_gazebo'
    descpkg = 'autocar_description'
    mappkg = 'autocar_map'

    world = os.path.join(get_package_share_directory(gzpkg), 'worlds', 'autocar.world')
    urdf = os.path.join(get_package_share_directory(descpkg),'urdf', 'autocar.xacro')
    rviz = os.path.join(get_package_share_directory(descpkg), 'rviz', 'view.rviz')
    
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    gzserver = os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')
    gzclient = os.path.join(pkg_gazebo, 'launch', 'gzclient.launch.py')
    
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT', '[{severity}]: {message}'
        ),

        SetEnvironmentVariable(
            'RCUTILS_COLORIZED_OUTPUT', '1'
        ),
   
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzserver), launch_arguments={'world': world}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzclient)
        ),
        
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'
        ),

        Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            output={'both': 'log'},
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            output={'both': 'log'}
        ),

        Node(
            package = mappkg,
            name = 'bof',
            executable = 'bof',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(gui))
    ])

def main():
    generate_launch_description()

if __name__ == '__main__':
    main()
