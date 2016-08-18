#!/bin/bash

export LANG=en_US.UTF-8
export LANGUAGE=en

read -r -p "Are you now in a directory where you want to store sarkofag repositories? [y/N] " response
case $response in
    [yY][eE][sS]|[yY]) 
        wget https://raw.githubusercontent.com/RCPRG-ros-pkg/sarkofag_robot/master/scripts/sarkofag.rosinstall -O /tmp/sark.rosinstall
	#wget https://raw.githubusercontent.com/RCPRG-ros-pkg/sarkofag_robot/master/scripts/update_and_compile.bash -O /tmp/update_and_compile.bash
	wget https://github.com/mwegiere/sarkofag_robot/blob/master/scripts/update_and_compile.bash -O /tmp/update_and_compile.bash
	wstool init
	bash /tmp/update_and_compile.bash
	touch robot/src/sarkofag_robot/scripts/hardware
        ;;
    *)
        echo "Run this script in a directory where you want to store sarkofag repositories"
        ;;
esac
