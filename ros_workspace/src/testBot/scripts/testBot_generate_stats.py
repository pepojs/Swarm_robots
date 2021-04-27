#!/usr/bin/python3

import sys
import enum
import math
import time
import json
import csv
import os

class Display:
    def __init__(self, file_name, file_stats_name):
        self.file_name = file_name
        self.file_stats_name = file_stats_name
        self.file_stats = open(self.file_stats_name, 'a')

        self.file = None
        self.line_offset = []
        self.line_counter = 0
        self.line_counter_tick = 8
        self.csv_reader = None

        self.number_of_robots = 0
        self.time = 0
        self.planning_cycles = 0

        self.prediction = False
        self.display_prediction = True

        self.cycles_list = []
        self.robots_with_pack = []
        self.current_cycle = -1
        self.cycles_with_pack = []
        self.path_length_list = []
        self.open_file()

        self.robot_positions = []
        self.robot_aims = []
        self.robot_colors = []
        self.robot_predictions = []
        self.robot_path_length = []
        self.pack_delivered = []

        self.start_pack_cycles = []
        self.pack_cycles_list = []

        for i in range(self.number_of_robots):
            self.pack_delivered.append(0)
            self.robot_positions.append([])
            self.robot_aims.append([])
            self.robot_colors.append([])
            self.robot_predictions.append([])

            self.cycles_with_pack.append([])
            self.path_length_list.append([])
            self.start_pack_cycles.append(0)

        self.pack_red_zone = 0
        self.pack_green_zone = 0
        self.pack_blue_zone = 0
        self.pack_yellow_zone = 0

        self.finished_test = False


    def __del__(self):
        if self.file != None:
            self.file.close()

        if self.file_stats != None:
            self.file_stats.close()

    def open_file(self):
        self.file = open(self.file_name, 'r', newline='')

        offset = 0
        for line in self.file:
            if line != '#finish':
                if offset == 0:
                    offset += len(line)
                else:
                    self.line_offset.append(offset)
                    offset += len(line)
        self.file.seek(0)

        self.csv_reader = csv.reader(self.file, delimiter=';')
        row = next(self.csv_reader)
        self.number_of_robots = int(row[0])
        if len(row) > 1:
            if row[1] == 'p':
                self.line_counter_tick += self.number_of_robots
                self.prediction = True


    def main_loop(self):
        while self.line_counter < len(self.line_offset)-1:
            row = next(self.csv_reader)
            self.time = float(row[0])

            row = next(self.csv_reader)
            self.planning_cycles = int(row[0])

            row = next(self.csv_reader)
            self.robot_path_length = [float(j) for j in row]

            row = next(self.csv_reader)
            for i in range(len(row)):
                self.robot_positions[i] = [float(j) for j in row[i].split(',')]

            row = next(self.csv_reader)
            for i in range(len(row)):
                self.robot_aims[i] = [float(j) for j in row[i].split(',')]

            row = next(self.csv_reader)
            for i in range(len(row)):
                self.robot_colors[i] = self.colors_to_int(row[i])

            row = next(self.csv_reader)
            for i in range(len(row)):
                self.pack_in_zone_update(row[i])

            row = next(self.csv_reader)
            self.pack_delivered = [int(j) for j in row]

            if self.prediction:
                for i in range(self.number_of_robots):
                    self.robot_predictions[i] = []
                    row = next(self.csv_reader)

                    for pos in row:
                        self.robot_predictions[i].append([float(j) for j in pos.split(',')])

            if self.current_cycle != self.planning_cycles:
                self.cycles_list.append(self.planning_cycles)
                self.current_cycle = self.planning_cycles

                for i in range(self.number_of_robots):
                    self.path_length_list[i].append(self.robot_path_length[i])

                temp = 0

                for i in range(len(self.robot_colors)):
                    if self.robot_colors[i] != 4 and self.start_pack_cycles[i] == 0:
                        self.start_pack_cycles[i] = self.planning_cycles

                    elif self.robot_colors[i] == 4 and self.start_pack_cycles[i] != 0:
                        self.pack_cycles_list.append(self.planning_cycles - self.start_pack_cycles[i])
                        self.start_pack_cycles[i] = 0

                    if self.robot_colors[i] != 4:
                        temp += 1
                        if len(self.cycles_with_pack[i]) > 0:
                            self.cycles_with_pack[i].append((self.cycles_with_pack[i][len(self.cycles_with_pack[i])-1] + 1)/self.current_cycle)
                        else:
                            self.cycles_with_pack[i].append(0)

                    else:
                        if len(self.cycles_with_pack[i]) > 0:
                            self.cycles_with_pack[i].append(self.cycles_with_pack[i][len(self.cycles_with_pack[i]) - 1]/self.current_cycle)
                        else:
                            self.cycles_with_pack[i].append(0)



                self.robots_with_pack.append(temp)

            self.line_counter += self.line_counter_tick

        if list(self.file)[-1] == '#finish':
            self.finished_test = True

        self.generate_stat()



    def pack_in_zone_update(self, row):
        temp = row.split(',')
        if temp[0] == 'red':
            self.pack_red_zone = int(temp[1])

        elif temp[0] == 'green':
            self.pack_green_zone = int(temp[1])

        elif temp[0] == 'blue':
            self.pack_blue_zone = int(temp[1])

        elif temp[0] == 'yellow':
            self.pack_yellow_zone = int(temp[1])

    def colors_to_int(self, color):
        color = color.split(',')
        if color[0] == 'r' or color[0] == 'red':
            return 0

        elif color[0] == 'g' or color[0] == 'green':
            return 1

        elif color[0] == 'b' or color[0] == 'blue':
            return 2

        elif color[0] == 'y' or color[0] == 'yellow':
            return 3

        else:
            return 4

    def generate_stat(self):
        self.file_stats.write(self.file_name+'\n')
        self.file_stats.write(str(self.time)+';'+str(self.cycles_list[-1]) + '\n')
        temp = str(self.robot_path_length[0])
        for i in self.robot_path_length[1:]:
            temp += ';'
            temp += str(i)

        self.file_stats.write(temp + '\n')

        temp = str(self.cycles_list[0])+','+str(self.robots_with_pack[0] / self.number_of_robots)
        for i in range(1, len(self.cycles_list)):
            temp += ';'
            temp += str(self.cycles_list[i])+','+str(self.robots_with_pack[i] / self.number_of_robots)

        self.file_stats.write(temp + '\n')

        temp = str(self.pack_cycles_list[0])
        for i in range(1, len(self.pack_cycles_list)):
            temp += ';'
            temp += str(self.pack_cycles_list[i])

        self.file_stats.write(temp + '\n')

        if self.finished_test:
            self.file_stats.write('finished' + '\n')
        else:
            self.file_stats.write('stopped' + '\n')


def main():
    file_name = ""
    file_stats_name = ""
    file_number = 0

    if len(sys.argv) == 3:
        file_name = sys.argv[1]
        file_stats_name = sys.argv[2]

    elif len(sys.argv) == 4:
        file_name = sys.argv[1]
        file_stats_name = sys.argv[2]
        file_number = int(sys.argv[3])

    else:
        print("Wrong number of scrypt argument, need file name with data")
        exit(-1)

    if file_number == 0:
        dis = Display(file_name, file_stats_name)
        dis.main_loop()

    else:
        file = open(file_stats_name, 'w')
        file.close()

        for i in range(file_number):
            file_name_list = file_name.split('_')
            new_name = file_name_list[0]+'_'+file_name_list[1]+'_'+file_name_list[2]+'_'+str(i)+'_'+file_name_list[4]+'_'+file_name_list[5]
            if os.path.exists(new_name):
                dis = Display(new_name, file_stats_name)
                dis.main_loop()
            else:
                continue



if __name__ == '__main__':
    main()
