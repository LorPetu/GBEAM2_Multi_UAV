import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

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
        parameters = [config_ground]
    ))

    ld.add_action(
        Node(
        package = 'gbeam2_ground',
        name = 'graph_draw',                  
        executable = 'graph_drawer',
        parameters = [config_ground]
    ))


    # Specify the path to your RViz configuration file
    rviz_config_file = "/home/pica/code/gbeam2_ws/src/gbeam2_simulator/rviz/turtlebot3_gazebo_poly&graph_draw_final.rviz"
    # rviz_config_file = "/home/ws/src/gbeam2_simulator/rviz/turtlebot3_gazebo_poly&graph_draw.rviz"

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
