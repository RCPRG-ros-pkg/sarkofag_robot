#!/bin/bash

if [ "x$ROS_MASTER_URI" != "x" ]
then
	echo "Run new consol without sourcing ROS"
	exit
else 

	re="robot/src/sarkofag_robot/scripts$"
	if [[ $PWD =~ $re ]];
	then 

		if test -e ./hardware; then
			wget https://raw.githubusercontent.com/RCPRG-ros-pkg/sarkofag_robot/master/scripts/sarkofag.rosinstall -O /tmp/sark.rosinstall
		else
			wget https://raw.githubusercontent.com/RCPRG-ros-pkg/sarkofag_robot/master/scripts/sarkofag_sim.rosinstall -O /tmp/sark.rosinstall
		fi

		#wget https://raw.githubusercontent.com/RCPRG-ros-pkg/sarkofag_robot/master/scripts/update_and_compile.bash -O /tmp/update_and_compile.bash
		wget https://raw.githubusercontent.com/mwegiere/sarkofag_robot/master/scripts/update_and_compile.bash -O /tmp/update_and_compile.bash


		cd ../../../../

		bash /tmp/update_and_compile.bash

	else  
		echo "You should run script from robot/src/sarkofag_robot/scripts directory"
		exit
	fi
fi
