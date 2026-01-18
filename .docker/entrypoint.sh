#!/bin/bash
set -e

# Source
echo "source /usr/share/bash-completion/completions/git" >> ~/.bashrc
# echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# git configuration
git config --global core.editor "code --wait"

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
