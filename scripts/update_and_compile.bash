#!/bin/bash
# Skrypt powinien być wołan z katalogu robot/src/sarkofag_robot/scripts


source /opt/ros/jade/setup.bash
export LANG=en_US.UTF-8
export LANG=en

wstool merge /tmp/sark.rosinstall
wstool update
cd underlay_isolated
catkin_make_isolated --install -DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install_isolated/setup.bash
cd ../underlay
catkin_make_isolated -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF --install
source install_isolated/setup.bash
cd ../robot
catkin_make_isolated -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=OFF --install
source devel_isolated/setup.bash
