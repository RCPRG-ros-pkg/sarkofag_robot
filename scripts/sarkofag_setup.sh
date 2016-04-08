#!/bin/bash

export LANG=en_US.UTF-8
export LANGUAGE=en

wget https://raw.githubusercontent.com/RCPRG-ros-pkg/sarkofag_robot/master/scripts/sarkofag.rosinstall -O /tmp/sark.rosinstall
wget https://raw.githubusercontent.com/RCPRG-ros-pkg/sarkofag_robot/master/scripts/update_and_compile.bash -O /tmp/update_and_compile.bash

if [ ! -d $1 ]; then
  mkdir $1
fi

cd $1
wstool init

bash /tmp/update_and_compile.bash
touch $1/robot/src/sarkofag_robot/scripts/hardware
