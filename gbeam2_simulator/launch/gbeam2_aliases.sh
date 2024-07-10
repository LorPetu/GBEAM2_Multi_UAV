alias cb='colcon build'
alias cb_gbeam2_controller='colcon build --packages-select gbeam2_controller --symlink-install'
alias cb_gbeam2_library='colcon build --packages-select gbeam2_library'
alias cb_gbeam2_interfaces='colcon build --packages-select gbeam2_interfaces'
alias cb_gbeam2_cooperation='colcon build --packages-select gbeam2_cooperation'
alias cb_gbeam2_communication='colcon build --packages-select gbeam2_communication'
alias cb_gbeam2_ground='colcon build --packages-select gbeam2_ground --symlink-install'
alias cb_gbeam2_simulator='colcon build --packages-select gbeam2_simulator'
alias gazebo_stage_4='ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py'
alias teleop='ros2 run turtlebot3_teleop teleop_keyboard'
alias poly_gen='ros2 launch gbeam2_controller polytope_gen.launch.py'
alias poly_draw='ros2 run gbeam2_ground poly_drawer'


