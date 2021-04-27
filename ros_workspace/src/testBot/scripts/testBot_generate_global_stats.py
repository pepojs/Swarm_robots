#!/usr/bin/python3

import sys
import enum
import math
import time
import json
import csv
import os

import matplotlib.pyplot as plt
import numpy as np

import pickle

class Display:
    def __init__(self, mode, file_dict):
        self.data = dict()
        self.mode = mode
        self.data_file_name = file_dict

    def main_loop(self):
        if self.data_file_name != '':
            if os.path.exists(self.data_file_name):
                try:
                    data_file = open(self.data_file_name, 'rb')
                    self.data = pickle.load(data_file)
                    data_file.close()

                except EOFError:
                    cwd = os.getcwd()
                    files_name = [f for f in os.listdir(cwd) if f.endswith('.stats')]

                    for file_name in files_name:
                        self.file = open(file_name, 'r', newline='')
                        self.csv_reader = csv.reader(self.file, delimiter=';')

                        self.generate_stat()
                        self.file.close()

                    data_file = open(self.data_file_name, 'wb')
                    pickle.dump(self.data, data_file)
                    data_file.close()



            else:
                cwd = os.getcwd()
                files_name = [f for f in os.listdir(cwd) if f.endswith('.stats')]

                for file_name in files_name:
                    self.file = open(file_name, 'r', newline='')
                    self.csv_reader = csv.reader(self.file, delimiter=';')

                    self.generate_stat()
                    self.file.close()

                data_file = open(self.data_file_name, 'wb')
                pickle.dump(self.data, data_file)
                data_file.close()

        else:
            cwd = os.getcwd()
            files_name = [f for f in os.listdir(cwd) if f.endswith('.stats')]

            for file_name in files_name:
                self.file = open(file_name, 'r', newline='')
                self.csv_reader = csv.reader(self.file, delimiter=';')

                self.generate_stat()
                self.file.close()

        self.display_stats()

    def generate_stat(self):

        while True:
            try:
                row = next(self.csv_reader)
                temp_name = row[0]

                row = next(self.csv_reader)
                temp_planning_cycles = int(row[1])

                row = next(self.csv_reader)
                temp_path_length = sum([float(j) for j in row])

                row = next(self.csv_reader)
                temp_cycles_list = []
                temp_robot_load = []
                for i in range(len(row)):
                    temp_cycles_list.append(float(row[i].split(',')[0]))
                    temp_robot_load.append(float(row[i].split(',')[1]))

                row = next(self.csv_reader)
                temp_pack_cycles = [float(j) for j in row]

                row = next(self.csv_reader)
                temp_final_status = row[0]

                if temp_name.split('_')[0] != 'scenario':
                    continue

                if self.mode == 'best' or self.mode == 'finish':
                    if temp_final_status == 'finished':
                        name_key = temp_name.split('_')

                        if not name_key[1] in self.data:
                            self.data[name_key[1]] = dict()

                        if not name_key[2] in self.data[name_key[1]]:
                            self.data[name_key[1]][name_key[2]] = {'time': [], 'length': [], 'cycles_list': [],
                                                                   'robot_load': [], 'pack_cycles': []}

                        self.data[name_key[1]][name_key[2]]['time'].append(temp_planning_cycles)
                        self.data[name_key[1]][name_key[2]]['length'].append(temp_path_length)
                        self.data[name_key[1]][name_key[2]]['cycles_list'].append(temp_cycles_list)
                        self.data[name_key[1]][name_key[2]]['robot_load'].append(temp_robot_load)
                        self.data[name_key[1]][name_key[2]]['pack_cycles'].append(temp_pack_cycles)

                else:
                    name_key = temp_name.split('_')

                    if not name_key[1] in self.data:
                        self.data[name_key[1]] = dict()

                    if not self.data[name_key[1]][name_key[2]] in self.data[name_key[1]]:
                        self.data[name_key[1]][name_key[2]] = {'time': [], 'length': [], 'cycles_list': [],
                                                               'robot_load': [], 'pack_cycles': []}

                    self.data[name_key[1]][name_key[2]]['time'].append(temp_planning_cycles)
                    self.data[name_key[1]][name_key[2]]['length'].append(temp_path_length)
                    self.data[name_key[1]][name_key[2]]['cycles_list'].append(temp_cycles_list)
                    self.data[name_key[1]][name_key[2]]['robot_load'].append(temp_robot_load)
                    self.data[name_key[1]][name_key[2]]['pack_cycles'].append(temp_pack_cycles)

            except StopIteration:
                break


    def display_stats(self):

        index = dict()

        for robots in sorted(self.data):
            index[robots] = dict()
            for packs in sorted(self.data[robots]):
                index[robots][packs] = self.data[robots][packs]['time'].index(min(self.data[robots][packs]['time']))
        robot_load = []
        cycles_list = []
        label = []

        for robots in sorted(self.data):
            for packs in sorted(self.data[robots]):
                cycles_list.append(self.data[robots][packs]['cycles_list'][index[robots][packs]])
                robot_load.append(self.data[robots][packs]['robot_load'][index[robots][packs]])
                label.append(robots+', '+packs)

        fig, ax = plt.subplots()

        for i in range(len(cycles_list)):
            plt.plot(cycles_list[i], robot_load[i], label=label[i])

        ax.set_ylabel('Obciążenie [%]')
        ax.set_xlabel('Czas [cykle planowania]')
        ax.set_title('Obciążenie robotów')
        ax.legend()
        plt.grid(True)


        labels = self.data.keys()
        bar_value = []
        bar_label = []
        pack_cycles_std = []
        param_list = ['time', 'length', 'pack_cycles_mean', 'pack_cycles_min', 'pack_cycles_max']
        titles = ['time', 'length', 'pack_cycles_mean', 'pack_cycles_min', 'pack_cycles_max']

        for j in range(len(param_list)):
            bar_value.append([])
            bar_label.append([])
            i = 0

            for robots in sorted(self.data):
                bar_value[j].append([])
                pack_cycles_std.append([])

                for packs in sorted(self.data[robots]):
                    if param_list[j] == 'pack_cycles_mean':
                        bar_value[j][i].append(np.mean(self.data[robots][packs]['pack_cycles'][index[robots][packs]]))
                        pack_cycles_std[i].append(np.std(self.data[robots][packs]['pack_cycles'][index[robots][packs]]))

                    elif param_list[j] == 'pack_cycles_min':
                        bar_value[j][i].append(np.min(self.data[robots][packs]['pack_cycles'][index[robots][packs]]))

                    elif param_list[j] == 'pack_cycles_max':
                        bar_value[j][i].append(np.max(self.data[robots][packs]['pack_cycles'][index[robots][packs]]))

                    else:
                        bar_value[j][i].append(self.data[robots][packs][param_list[j]][index[robots][packs]])
                    bar_label[j].append(packs)

                i += 1


        for j in range(len(param_list)):
            plt.figure(j)
            x = np.arange(len(labels))  # the label locations
            width = 0.6  # the width of the bars

            fig, ax = plt.subplots()
            rects = []
            for i in range(len(bar_value[j])):
                if param_list[j] == 'pack_cycles_mean':
                    rects.append(ax.bar((x - width / 2) + (i + 0.5) * (width / len(bar_value[j])), bar_value[j][i],
                                         width / len(bar_value[j]), label=bar_label[j][i]))

                    ax.errorbar((x - width / 2) + (i + 0.5) * (width / len(bar_value[j])), bar_value[j][i],
                                         yerr=pack_cycles_std[i], color='black', fmt='o', markersize=8, capsize=20)

                else:
                    rects.append(ax.bar((x - width/2) + (i+0.5)*(width / len(bar_value[j])), bar_value[j][i],
                                        width / len(bar_value[j]), label=bar_label[j][i]))

            # Add some text for labels, title and custom x-axis tick labels, etc.
            ax.set_ylabel('Scores')
            ax.set_title(titles[j])
            ax.set_xticks(x)
            ax.set_xticklabels(labels)
            ax.legend()

            for rect in rects:
                ax.bar_label(rect, padding=3)

            #fig.tight_layout()

        plt.show()

def main():
    mode = 'best'
    file_dict = ''

    if len(sys.argv) == 1:
        pass

    elif len(sys.argv) == 2:
        mode = sys.argv[1]

    elif len(sys.argv) == 3:
        mode = sys.argv[1]
        file_dict = sys.argv[2]

    else:
        print("Wrong number of scrypt argument, need file name with data")
        exit(-1)

    dis = Display(mode, file_dict)
    dis.main_loop()

if __name__ == '__main__':
    main()
