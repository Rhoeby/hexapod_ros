#!/bin/bash

# the new BT-210
sudo rfcomm bind /dev/rfcomm0 B8:63:BC:00:1F:8C 
sudo chmod 666 /dev/rfcomm0

sudo chmod 666 /dev/ttyS1

sudo killall static_transform_publisher
sudo killall roscore

roscore &

