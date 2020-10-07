
# RoverRobotics Naviigatioin

How to use:
1. Install dependencies

    `sudo apt install ros-melodic-rr-* ros-melodic-map-server ros-melodic-move-base ros-melodic-amcl ros-melodic-dwa-local-planner ros-melodic-gmapping ros-melodic-teleop-twist-keyboard`

2. Run RoverRobotics Gazebo Simulator

    `roslaunch rr_openrover_simulation 4wd_rover_gazebo.launch`

3. Run navigation script

    `roslaunch rr_navigation rr_4wd_nav_sim.launch`

4. Run rViz

    `roslaunch rr_navigation rviz_navigation.launch`