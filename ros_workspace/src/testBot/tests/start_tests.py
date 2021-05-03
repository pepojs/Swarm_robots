#!/usr/bin/python3

import time
import subprocess
import sys
import signal

test_counter = 1
file = open('test_log.txt', 'w')

if len(sys.argv) == 4:
    pass

elif len(sys.argv) == 5:
    test_counter = int(sys.argv[4])

else:
    print("Wrong number of arguments")
    exit(-1)

for k in range(test_counter):        
    pid_argos = subprocess.Popen(["xterm", "-e", "argos3", "-c", sys.argv[1]])
    time.sleep(10)

    pid_robots = []
    number_of_loops = int(int(sys.argv[2])/4.0)
    rest_of_division = int(sys.argv[2])%4
    robots_counter = 1

    #pid_robots.append(subprocess.Popen(["xterm", "-e", "./run_robots.py", str(robots_counter), str(robots_counter+int(sys.argv[2])-1)]))
    
    for i in range(number_of_loops):
        pid_robots.append(subprocess.Popen(["xterm", "-e", "./run_robots.py", str(robots_counter), str(robots_counter+3)]))
        time.sleep(8)
        robots_counter += 4

    if rest_of_division != 0:
        pid_robots.append(subprocess.Popen(["xterm", "-e", "./run_robots.py", str(robots_counter), str(robots_counter+rest_of_division-1)]))
    
    time.sleep(10)

    pid_xdotool = subprocess.run(["xdotool", "search", "--name", "ARGoS v3.0.0-beta56"], stdout=subprocess.PIPE, universal_newlines=True)
    window_ID = pid_xdotool.stdout

    pid_xdotool = subprocess.run(["xdotool", "getwindowgeometry", window_ID], stdout=subprocess.PIPE, universal_newlines=True)
    window_geometry = pid_xdotool.stdout

    pos = window_geometry.split('\n')[1].split(' ')[3].split(',')
    x_pos = pos[0]
    y_pos = pos[1]

    pid_xdotool = subprocess.run(["xdotool", "mousemove", str(int(x_pos)+141), str(int(y_pos)+5)], stdout=subprocess.PIPE, universal_newlines=True)
    pid_xdotool = subprocess.run(["xdotool", "click", "--window", window_ID, "1"], stdout=subprocess.PIPE, universal_newlines=True)

    time.sleep(1)

    data_output_name = sys.argv[1].split('/')
    

    if sys.argv[3] == "MPC":
        data_output_name = data_output_name[len(data_output_name)-1].split('.')[0]+"_"+str(k)+"_MPC.data"
        try:
            pid_controler = subprocess.run(["rosrun", "testBot", "testBot_controller_global_loc_mpc_M.py", sys.argv[1], data_output_name])#, timeout=2100)
            file.write("Test {} finished! ".format(k))
            
        except KeyboardInterrupt:
            pid_argos.kill()
            for pid in pid_robots:
                pid.kill()
            
            file.write("Test {} break! \n".format(k))
            
        except subprocess.TimeoutExpired:
            pid_argos.kill()
            for pid in pid_robots:
                pid.kill()

            file.write("Test {} timeout! \n".format(k))

    else:
        data_output_name = data_output_name[len(data_output_name)-1].split('.')[0]+"_"+str(k)+"_A_star.data"
        try:
            pid_controler = subprocess.run(["rosrun", "testBot", "testBot_controller_global_loc.py", sys.argv[1], data_output_name])
            file.write("Test {} finished! ".format(k))
            
        except KeyboardInterrupt:
            pid_argos.kill()
            for pid in pid_robots:
                pid.kill()

            file.write("Test {} break! \n".format(k))
                
        except subprocess.TimeoutExpired:
            pid_argos.kill()
            for pid in pid_robots:
                pid.kill()

            file.write("Test {} timeout! \n".format(k))
            
    pid_argos.kill()
    for pid in pid_robots:
        pid.kill()

file.close()
