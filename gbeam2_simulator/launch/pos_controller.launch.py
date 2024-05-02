import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config_sim = os.path.join(
    get_package_share_directory('gbeam2_simulator'),
    'config',
    'ddrive_param.yaml'
    )

    ld.add_action(
        Node(
        package = 'gbeam2_simulator',
        name = 'pos_contr',                  
        executable = 'ddrive_position',
        parameters = [config_sim]
    ))

    



    ##ld.add_action(node)
    return ld