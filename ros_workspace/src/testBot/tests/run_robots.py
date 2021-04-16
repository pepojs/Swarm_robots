#!/usr/bin/python3

import sys
import subprocess
import time

start = 1
bots = []

if len(sys.argv) == 2:
    for i in range(start, int(sys.argv[1])):
        bots.append(subprocess.Popen(["rosrun", "testBot", "testBot_robot_global_loc_pause_mpc.py", str(i)]))
        time.sleep(1)

    try:
        bots.append(subprocess.run(["rosrun", "testBot", "testBot_robot_global_loc_pause_mpc.py", str(int(sys.argv[1]))]))

    except KeyboardInterrupt:
        for bot in bots:
            bot.kill()

    
elif len(sys.argv) == 3:
    for i in range(int(sys.argv[1]), int(sys.argv[2])):
        bots.append(subprocess.Popen(["rosrun", "testBot", "testBot_robot_global_loc_pause_mpc.py", str(i)]))
        time.sleep(1)

    try:
        bots.append(subprocess.run(["rosrun", "testBot", "testBot_robot_global_loc_pause_mpc.py", str(int(sys.argv[2]))]))

    except KeyboardInterrupt:
        for bot in bots:
            bot.kill()

    

else:
    print("Wrong number of arguments !")
    exit(-1)
        
for bot in bots:
    bot.kill()
