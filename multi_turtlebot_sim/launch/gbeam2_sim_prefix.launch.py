import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch.actions import LogInfo, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessIO

## Launch file description
# This launch file starts the visualization part of gbeam2_ground within rviz2
# Then starts in order: polytope_generation_node --> graph_update_node --> set mapping status true --> exploration_node
# It could happens that the printout of the node does not appear in the order depicted above, this is not a problem


def generate_launch_description():
    robot_prefix_arg = DeclareLaunchArgument('robot_prefix', default_value='')
    robot_prefix = LaunchConfiguration('robot_prefix')
    

    graph_merger = Node(
        package='gbeam2_communication',
        name='partial_graph_merger',
        executable='partial_graph_merger',
        namespace=robot_prefix,
        parameters=[os.path.join(
    get_package_share_directory('gbeam2_communication'),
    'config',
    'communication_param.yaml'
    )]
    )

    coop_node = Node(
        package='gbeam2_cooperation',
        name='coop_manager',
        executable='cooperation_manager',
        namespace=robot_prefix,
        parameters=[os.path.join(
    get_package_share_directory('gbeam2_cooperation'),
    'config',
    'cooperation_param.yaml'
    )]
    )
    
    ddrive = Node(
        package='gbeam2_simulator',
        name="pos_contr",
        executable='ddrive_position',
        namespace=robot_prefix,
        parameters=[os.path.join(
    get_package_share_directory('gbeam2_simulator'),
    'config',
    'ddrive_param.yaml'
    )]
    )




    ld = LaunchDescription(
        [
        ddrive,       
        graph_merger,
        coop_node
        ]
    )


    config_ground = os.path.join(
        get_package_share_directory('gbeam2_ground'),
        'config',
        'ground_param.yaml'
        )
    # rviz_config_file = os.path.join(
    # get_package_share_directory('gbeam2_simulator'),
    # 'rviz',
    # 'turtlebot3_gazebo_poly&graph_draw.rviz"'
    # )

    ld.add_action(
        Node(
        package = 'gbeam2_ground',
        name = 'poly_draw',                  
        executable = 'poly_drawer',
        namespace=robot_prefix,
        parameters = [config_ground]
    ))

    ld.add_action(
        Node(
        package = 'gbeam2_ground',
        name = 'graph_draw',                  
        executable = 'graph_drawer',
        namespace=robot_prefix,
        parameters = [config_ground]
    ))

    ld.add_action(robot_prefix_arg)

    return ld