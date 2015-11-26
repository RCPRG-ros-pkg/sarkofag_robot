#!/bin/bash
# Skrypt powinien być wołan z katalogu robot/src/sarkofag_robot/scripts

if [ "x$ROS_MASTER_URI" != "x" ]
then
	echo "Ta konsola jest skazona, otworz nowa bez source'owania zadnych plikow ROSa"
	exit
fi

wget https://raw.githubusercontent.com/RCPRG-ros-pkg/sarkofag_robot/master/scripts/sarkofag_sim.rosinstall -O /tmp/sark.rosinstall
wget https://raw.githubusercontent.com/RCPRG-ros-pkg/sarkofag_robot/master/scripts/update_and_compile.bash -O /tmp/update_and_compile.bash


cd ../../../../

bash /tmp/update_and_compile.bash