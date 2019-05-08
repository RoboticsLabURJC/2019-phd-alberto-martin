#!/bin/bash

source "/opt/ros/$ROS_DISTRO/setup.bash"
export TURTLEBOT3_MODEL=waffle_pi

input_file_ext="${1#*.}"


if [ "$input_file_ext" = 'launch' ]; then

    if [ "$2" = 'gzweb' ]; then
        cd $GZWEB_WS
        roslaunch $input_file_ext & npm start
    fi

    roslaunch $input_file_ext
else
    exec "$@"
fi


