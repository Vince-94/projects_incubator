#!/bin/bash
# -----------------------------------------------------------------------------
# Copyright (c) 2025 Ruotolo Vincenzo. All rights reserved.
#
# This software is proprietary and licensed, not sold. See the LICENSE.md file
# in the project root for the full license terms.
#
# Unauthorized copying, distribution, modification, or resale of this file,
# via any medium, is strictly prohibited without prior written permission.
# -----------------------------------------------------------------------------
set -e

echo "Entrypoint"

# Source
echo "source /usr/share/bash-completion/completions/git" >> ~/.bashrc
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# git configuration
git config --global core.editor "code --wait"

# Source ROS2 workspace
if [[ -d "../install" ]]; then
	source install/local_setup.bash  # TODO to check
fi

# Ubuntu info
if ! [[ $USER == $CONTAINER_USR ]] && [[ $UID == $CONTAINER_UID ]]; then
	echo "User is not set correctly!"
	if ! [[ $USER == $CONTAINER_USR ]]; then
		echo "Username mismatch: $USER is not $CONTAINER_USR"
	else
		echo "UID mismatch: $UID is not $CONTAINER_UID"
	fi
	exit
fi


# Source
source ~/.bashrc

if [[ -n "$CI" ]]; then
    exec /bin/bash
else
    exec "$@"
fi
