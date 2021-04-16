#!/bin/bash
gnome-terminal -- argos3 -c scenario_r4_p8.argos &
echo $! >tmp_pid.txt
sleep 10
kill -USR1 $1
wait

#TIMEOUT=40
#SUBSTRING='The physics engine'
#COMMAND="argos3 -c scenario_r4_p8.argos"
#expect -c "set echo \"-noecho\"; set timeout $TIMEOUT; spawn -noecho $COMMAND;# \
#expect \"$SUBSTRING\" {exec kill -USR1 $1} timeout { puts \"\nTimed out!\"; ex#it 1 }"
