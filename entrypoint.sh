#!/bin/bash
set -e
source "/home/$USERNAME/workspace/$APP_NAME/setup.bash"
if [[ -f "/usr/share/$GAZEBO_VERSION/setup.sh" ]]
then
    source /usr/share/$GAZEBO_VERSION/setup.sh
fi
printenv
exec "${@:1}"