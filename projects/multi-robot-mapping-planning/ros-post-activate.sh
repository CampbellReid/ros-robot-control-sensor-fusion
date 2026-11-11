#!/bin/bash

# Source ROS Noetic setup
source /opt/ros/noetic/setup.bash

# Source the workspace setup
source /home/campbell/repos/3806ICT-Assignment-3-Kenneth-Campbell/devel/setup.bash

# Set TURTLEBOT3 model
export TURTLEBOT3_MODEL=burger

# Update PYTHONPATH to include local site-packages and conda environment
export PYTHONPATH=/home/campbell/repos/3806ICT-Assignment-3-Kenneth-Campbell/src/assignment3/scripts:/home/campbell/miniconda3/envs/ros_noetic/lib/python3.8/site-packages:$PYTHONPATH

# Update PATH to include conda environment
export PATH=/home/campbell/miniconda3/envs/ros_noetic/bin:$PATH