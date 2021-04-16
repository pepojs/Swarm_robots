#!/bin/bash

START=1
 
for (( c=$START; c<$1; c++ ))
do
    rosrun testBot testBot_robot_global_loc_pause_mpc.py $c &
    sleep 1
done

rosrun testBot testBot_robot_global_loc_pause_mpc.py $1

#rosrun testBot testBot_robot_global_loc_pause_mpc.py
