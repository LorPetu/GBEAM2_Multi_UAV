import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():
    # Launch configuration variables specific to simulation
    robot_prefix_arg = DeclareLaunchArgument('robot_prefix', default_value='')
    robot_prefix = LaunchConfiguration('robot_prefix')

    lidar_height_arg = DeclareLaunchArgument('lidar_height', default_value='0.122')
    lidar_height = LaunchConfiguration('lidar_height')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='')
    use_sim_time = LaunchConfiguration('use_sim_time')

    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    x_pose = LaunchConfiguration('x_pose')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    y_pose = LaunchConfiguration('y_pose')


    # Obtain urdf from xacro files.
    multi_turtlebot_sim_pkg_dir = get_package_share_directory('multi_turtlebot_sim')
    xacro_file_path = os.path.join(multi_turtlebot_sim_pkg_dir, 'urdf', 'turtlebot3_waffle.urdf.xacro')
    robot_desc = Command(['xacro ', str(xacro_file_path), ' frame_prefix:=', robot_prefix, ' topic_prefix:=', robot_prefix, ' lidar_height_arg:=',lidar_height])

    # Robot state publisher
    # This node will take the urdf description and:
    # - publish the transforms using the prefix set by the frame_prefix parameter.
    # - publish the robot description under the set namespace.
    # - subscribe to joint states under the set namespace.
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=robot_prefix,
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix':
                    PythonExpression(["'", LaunchConfiguration('robot_prefix'), "/'"])
            }],
        )
    
    # Create a tf from world to robot_prefix/odom
    world_static_tf = ExecuteProcess( 
        cmd=[[
            # executable
            'ros2 run tf2_ros static_transform_publisher ',
            # parameters
            '0 0 0 0 0 0 world ',
            PythonExpression(["'/", LaunchConfiguration('robot_prefix'), "/odom'"])
        ]],
        shell=True
    )

    # Spawn robot
    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', PathJoinSubstitution([robot_prefix, 'waffle']),
            '-topic', PathJoinSubstitution([robot_prefix, 'robot_description']),
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )



    # launch gbeam 2 os.path.join(get_package_share_directory('gbeam2_simulator'),'launch/ground_nodes.launch.py')

    gbeam2_launch = GroupAction(
        actions=[

            SetRemap(src='/scan',dst= PythonExpression(["'/", LaunchConfiguration('robot_prefix'), "/scan'"])),

            IncludeLaunchDescription(
                 PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('multi_turtlebot_sim'),
                         'launch/gbeam2_sim_prefix.launch.py')),
                 launch_arguments = {
                       'robot_prefix' : robot_prefix

                 }.items(),

            )
        ]
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(lidar_height_arg)
    ld.add_action(robot_prefix_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_state_publisher)
    ld.add_action(world_static_tf)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    ld.add_action(gbeam2_launch)

    return ld
