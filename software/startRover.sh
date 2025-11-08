#!/bin/bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /home/makemorerobot/Rover_2023_2024/software/install/setup.bash
sleep 15
ros2 launch rover2_main rover2_main_launch.py 2> /home/makemorerobot/Rover_2023_2024/software/output.txt

