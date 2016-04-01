cmake_minimum_required(VERSION 2.8.3)
project(sark_bringup)
find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS xacro)

foreach(dir config launch)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

 install(PROGRAMS scripts/sark_multi_trajectory.py scripts/sark_simple_trajectory.py scripts/sark_synchro_joint.py scripts/sark_synchro_motor.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
