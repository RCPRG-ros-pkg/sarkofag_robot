#!/bin/bash
# Skrypt powinien być wołan z katalogu underlay/src/sarkofag_robot

if [ "x$ROS_MASTER_URI" != "x" ]
then
	echo "Ta konsola jest skazona, otworz nowa bez source'owania zadnych plikow ROSa"
	exit
fi

source /opt/ros/indigo/setup.bash
export LANG=en

cd ../../../

wget https://raw.githubusercontent.com/RCPRG-ros-pkg/RCPRG_rosinstall/master/sarkofag.rosinstall -O /tmp/sarkofag.rosinstall


wstool merge /tmp/sarkofag.rosinstall
wstool up
cd underlay_isolated
catkin_make_isolated --install -DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install_isolated/setup.bash
cd ../underlay
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF
source devel/setup.bash
