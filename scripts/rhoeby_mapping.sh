#!/bin/bash

export LIBGL_ALWAYS_SOFTWARE=1

source ../../../devel/setup.bash

roslaunch ../launch/rhoeby_gmapping.launch model:=../urdf/hexapod_01.urdf &

rosrun hexapod_ros hexapod_ros _phone_port_name:=/dev/ttyS1

sudo killall roslaunch




