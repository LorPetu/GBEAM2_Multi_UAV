import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('gbeam2_controller'),
        'config',
        'global_param.yaml'
        )

    ld.add_action(
        Node(
        package = 'gbeam2_controller',
        name = 'poly_gen',                  ##qua il node name è poly_gen, il nome che viene dato alla classe nel polytope_generation_node
        executable = 'polytope_generation_node',
        parameters = [config]
    ))

    ld.add_action(
        Node(
        package = 'gbeam2_controller',
        name = 'graph_update',                  
        executable = 'graph_update_node',
        parameters = [config],
        # depends_on = ['poly_gen']
    ))

    # ld.add_action(
    #     Node(
    #     package = 'gbeam2_controller',
    #     name = 'graph_expl',                 
    #     executable = 'exploration_node',
    #     parameters = [config],
    #     # depends_on = ['graph_update']
    # ))

    return ld