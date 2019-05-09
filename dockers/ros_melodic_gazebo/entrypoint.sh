#!/bin/bash

source "/opt/ros/$ROS_DISTRO/setup.bash"
export TURTLEBOT3_MODEL=waffle_pi

input_file_ext="${1#*.}"


if [ "$input_file_ext" = 'launch' ]; then
    roscore &
    if [ "$2" = 'gzweb' ]; then
        cd $GZWEB_WS
        roslaunch $1 & npm start
        echo "roslaunch $1 & npm start"
        exit 0
    fi

    roslaunch $1
else
    exec "$@"
fi


