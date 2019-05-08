#!/bin/bash

source "/opt/ros/$ROS_DISTRO/setup.bash"
export PYTHONPATH=/opt/ros/melodic/lib/python3/dist-packages/

exec "$@"