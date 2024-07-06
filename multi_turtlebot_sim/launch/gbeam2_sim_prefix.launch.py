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
    
    config = os.path.join(
    get_package_share_directory('gbeam2_controller'),
    'config',
    'global_param.yaml'
    )

    poly_gen=Node(
        package = 'gbeam2_controller',
        name = 'poly_gen',                  ##qua il node name è poly_gen, il nome che viene dato alla classe nel polytope_generation_node
        executable = 'polytope_generation_node',
        parameters = [config],
        namespace= robot_prefix
    )

    graph_update=Node(
        package = 'gbeam2_controller',
        name = 'graph_update',                  
        executable = 'graph_update_node',
        parameters = [config],
        namespace=robot_prefix,
        # depends_on = ['poly_gen']
    )

    # graph_expl = Node(
    #     package = 'gbeam2_controller',
    #     name = 'graph_expl',                 
    #     executable = 'exploration_node',
    #     parameters = [config],
    #     namespace=robot_prefix,
    #     # depends_on = ['graph_update']
    # )
    
    # ddrive = Node(
    #     package='gbeam2_simulator',
    #     name="pos_contr",
    #     executable='ddrive_position',
    #     parameters=[os.path.join(
    # get_package_share_directory('gbeam2_simulator'),
    # 'config',
    # 'ddrive_param.yaml'
    # )]
    # )

    # ld = LaunchDescription(
    #     [
    #     visualization_launch,
    #     poly_gen,
         
    #     RegisterEventHandler(OnProcessStart(
    #          target_action=poly_gen, on_start=[graph_update, ddrive])), 
         
    #     RegisterEventHandler(OnProcessStart(
    #          target_action=graph_update,
    #          on_start=[LogInfo(msg="Started the graph_update"),
    #                                set_map_status,
    #                                LogInfo(msg="Set mapping status"),
    #                                     graph_expl]
    #             )
    #     )
    #     ]
    # )


    ld = LaunchDescription(
        [
        poly_gen,
         
        RegisterEventHandler(OnProcessStart(
             target_action=poly_gen, on_start=[graph_update])), 
         
        RegisterEventHandler(OnProcessStart(
             target_action=graph_update,
             on_start=LogInfo(msg="Started the graph_update")
                    
                )
        )
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