###############################################################################################################################################
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
        package = 'gbeam2_ground',
        name = 'poly_draw',                  ##qua il node name è poly_gen, il nome che viene dato alla classe nel polytope_generation_node
        executable = 'poly_drawer',
        parameters = [config]
    ))
    # Specify the path to your RViz configuration file
    rviz_config_file = "/home/pica/code/gbeam2_ws/src/gbeam2_simulator/rviz/turtlebot3_gazebo_poly_draw.rviz"

    ld.add_action(
        Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen"
    )
    )

    

    
    ##ld.add_action(node)
    return ld


############################################################################################################################