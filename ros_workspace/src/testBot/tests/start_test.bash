#!/bin/bash

#handler_argos()
#{
#    echo "Argos run"
#    quit=1
#}

#trap handler_argos USR1

argos3 -c $1 &
pid_argos=$!
sleep 5

./run_robots.bash $2 &
pid_run1=$!
sleep 15

#quit=0
#while [ "$quit" -ne 1 ]; do
#    sleep 1
#done

windowID=$(xdotool search --name 'ARGoS v3.0.0-beta56')

temp=$(xdotool getwindowgeometry $windowID |grep 'Position')
pos=$(echo $temp | cut -d' ' -f 2)

x_win=$(echo $pos | cut -d',' -f 1)
y_win=$(echo $pos | cut -d',' -f 2)

xdotool mousemove $(($x_win+141)) $(($y_win+5))
xdotool click 1

name=$(echo $1 | cut -d'.' -f 1)

sleep 2

if [ "$3" = "MPC" ]; then
    rosrun testBot testBot_controller_global_loc_mpc_M.py $1 "${name}.csv"
else
    rosrun testBot testBot_controller_global_loc.py $1 "${name}.csv"
fi

sleep 2
echo "Stop"
kill -9 $pid_argos
kill -9 $pid_run1
