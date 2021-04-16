#!/usr/bin/python3

import sys
import enum
import math
import time
import json
import csv
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Rectangle

class Display:
    def __init__(self, file_name):
        self.time_delay = 20
        self.time_delay_min = 1
        self.time_delay_max = 100

        self.file_name = file_name

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

        self.open_file()

        self.robot_positions = []
        self.robot_aims = []
        self.robot_colors = []
        self.robot_predictions = []
        self.robot_path_length = []
        self.pack_delivered = []
        for i in range(self.number_of_robots):
            self.pack_delivered.append(0)
            self.robot_positions.append([])
            self.robot_aims.append([])
            self.robot_colors.append([])
            self.robot_predictions.append([])

        self.pack_black_zone = []
        for i in range(4):
            self.pack_black_zone.append([])
            for j in range(4):
                self.pack_black_zone[i].append(0)

        self.pack_red_zone = 0
        self.pack_green_zone = 0
        self.pack_blue_zone = 0
        self.pack_yellow_zone = 0

        plt.ion()
        self.fig = plt.figure(constrained_layout=False)
        self.gs = self.fig.add_gridspec(ncols=5, nrows=self.number_of_robots)
        self.key = None
        self.run = True
        self.finish = False
        self.change_state = False

        self.fig.canvas.mpl_connect('key_press_event', self.press)
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        plt.grid(True)
        plt.show()

    def __del__(self):
        if self.file != None:
            self.file.close()

    def open_file(self):
        self.file = open(self.file_name, 'r', newline='')

        offset = 0
        for line in self.file:
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

    def on_close(self, event):
        self.finish = True

    def press(self, event):
        print('press ', event.key)
        self.key = event.key

        if self.key == 'enter':
            self.run = not self.run

        if self.key == 'escape':
            self.finish = True

        if self.key == 'left':
            if not self.run:
                if self.line_counter - self.line_counter_tick >= 0:
                    self.line_counter -= self.line_counter_tick

                self.file.seek(self.line_offset[self.line_counter])
                print('Line: ', self.line_counter)
                self.change_state = True

        if self.key == 'right':
            if not self.run:
                if self.line_counter + self.line_counter_tick < len(self.line_offset)-1:
                    self.line_counter += self.line_counter_tick
                    self.file.seek(self.line_offset[self.line_counter])

                print('Line: ', self.line_counter)
                self.change_state = True

        if self.key == 'up':
            if self.time_delay >= self.time_delay_min + 1:
                self.time_delay -= 1
                print('Time_delay: ', self.time_delay)

        if self.key == 'down':
            if self.time_delay <= self.time_delay_max - 0.1:
                self.time_delay += 1
                print('Time_delay: ', self.time_delay)

        if self.key == 'p':
            if self.prediction:
                self.display_prediction = not self.display_prediction
                self.display_map()

    def main_loop(self):
        while not self.finish:
            plt.pause(.0001)

            while self.run and not self.finish:
                if self.line_counter < len(self.line_offset)-1:
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

                    self.line_counter += self.line_counter_tick
                self.display_map()

                for i in range(self.time_delay):
                    if self.finish:
                        break
                    else:
                        plt.pause(.01)

            if not self.run and self.change_state:
                self.change_state = False

                if self.line_counter < len(self.line_offset)-1:
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

                    self.file.seek(self.line_offset[self.line_counter])
                self.display_map()

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

    def display_map(self):
        min_x = -3
        max_x = 3
        min_y = -3
        max_y = 3

        color_list = ['red', 'green', 'blue', 'yellow', 'black']

        ax_robots = []

        self.fig.clear()

        for i in range(self.number_of_robots):
            ax_robots.append(self.fig.add_subplot(self.gs[i, 0]))
            ax_robots[i].plot(1, 1, "or", markersize=(80/self.number_of_robots), color=color_list[self.robot_colors[i]])
            ax_robots[i].text(0.93, 1.01, "{:^}".format(i+1), size=(40/self.number_of_robots), color='purple')
            ax_robots[i].text(0.93, 0.989, "{:<5.2f}".format(self.robot_path_length[i]), size=(40 / self.number_of_robots))
            ax_robots[i].text(0.96, 1.01, "{:<}".format(self.pack_delivered[i]),
                              size=(40 / self.number_of_robots))
            ax_robots[i].axis('off')

        ax = self.fig.add_subplot(self.gs[:, 1:])
        ax.set_aspect(1)

        if self.prediction and self.display_prediction:
            for i in range(self.number_of_robots):
                for j in range(len(self.robot_predictions[i])):
                    ax.plot(self.robot_predictions[i][j][1], self.robot_predictions[i][j][0], "or",
                            color='#00'+format(255-int(j*(255.0/len(self.robot_predictions[i]))), '02x') \
                                  + format(255-int(i*(255.0/self.number_of_robots)), '02x'), alpha=0.8)


        ax.plot([(i[0], i[1])[1] for i in self.robot_positions],
                [(i[0], i[1])[0] for i in self.robot_positions],
                "or", markersize=15, color='purple')

        ax.plot([(i[0], i[1])[1] for i in self.robot_aims],
                [(i[0], i[1])[0] for i in self.robot_aims],
                "or", markersize=15, color='teal')

        for i in range(len(self.robot_positions)):
            ax.text(self.robot_positions[i][1]+0.09, self.robot_positions[i][0]-0.06, "{}".format(i+1))

        points = [[1.2, 2.7], [-1.2, 2.7], [1.2, 2.25], [-1.2, 2.25], [1.2, 1.8], [-1.2, 1.8], [2.85, 1.2],
                  [-2.85, 1.2],
                  [2.85, 0.75], [0.3, 0.75], [-0.75, 0.75], [-2.85, 0.75], [2.85, 0.3], [1.2, 0.3], [-0.75, 0.3],
                  [-2.85, 0.3], [2.85, -0.3], [0.75, -0.3],
                  [-1.2, -0.3], [-2.85, -0.3], [2.85, -0.75], [0.75, -0.75], [-0.3, -0.75], [-2.85, -0.75],
                  [2.85, -1.2],
                  [-2.85, -1.2], [1.2, -1.8], [-1.2, -1.8], [1.2, -2.25], [-1.2, -2.25], [1.2, -2.7], [-1.2, -2.7],
                  [2.85, 1.2], [2.85, -1.2], [2.4, 1.2], [2.4, -1.2],
                  [1.8, 1.2], [1.8, -1.2], [1.2, 2.7], [1.2, -2.7], [0.75, 2.7], [0.75, 0.75], [0.75, -0.3],
                  [0.75, -2.7], [0.3, 2.7], [0.3, 0.75], [0.3, -1.2], [0.3, -2.7], [-0.3, 2.7], [-0.3, 1.2],
                  [-0.3, -0.75], [-0.3, -2.7], [-0.75, 2.7], [-0.75, 0.3],
                  [-0.75, -0.75], [-0.75, -2.7], [-1.2, 2.7], [-1.2, -2.7], [-1.8, 1.2], [-1.8, -1.2], [-2.4, 1.2],
                  [-2.4, -1.2], [-2.85, 1.2], [-2.85, -1.2]]

        for i in range(0, len(points), 2):
            ax.plot([points[i][1], points[i + 1][1]], [points[i][0], points[i + 1][0]], 'b-', alpha=0.25)

        ax.add_patch(Rectangle((1.35, 1.3), 1.4, 1.5, color='blue'))
        ax.text(2.75, 2.6, "{}".format(self.pack_blue_zone), color='white')

        ax.add_patch(Rectangle((-2.7, 1.3), 1.4, 1.5, color='yellow'))
        ax.text(-1.3, 2.6, "{}".format(self.pack_yellow_zone), color='black')

        ax.add_patch(Rectangle((1.35, -2.8), 1.4, 1.5, color='red'))
        ax.text(2.75, -1.5, "{}".format(self.pack_red_zone), color='white')

        ax.add_patch(Rectangle((-2.7, -2.8), 1.4, 1.5, color='green'))
        ax.text(-1.3, -1.5, "{}".format(self.pack_green_zone), color='white')


        '''
        pos_x = [0.1, 1.1, 0.6, -0.4]
        pos_y = [-0.35, 0.1, 1.1, 0.6]

        for i in range(4):
            for j in range(4):
                ax.plot(pos_x[i], pos_y[i]-0.2*j, "or", color=color_list[j])
                ax.text(pos_x[i]-0.1, -0.05+pos_y[i]-0.2*j, "{}".format(self.pack_black_zone[i][j]))

        '''

        ax.set_xlim([min_x, max_x])
        ax.set_ylim([min_y, max_y])

        ax.invert_xaxis()

        ax.text(3.0, 3.2, "Time: {:<5.4f}".format(float(self.time)))
        ax.text(0, 3.2, "Path length: {:>5.4f}".format(sum(self.robot_path_length)))
        ax.text(3.0, 3.6, "File: " + self.file_name)
        ax.text(0, 3.6, "Planning cycles: {}".format(self.planning_cycles))

        plt.pause(0.001)

def main():
    file_name = ""

    if len(sys.argv) != 2:
        print("Wrong number of scrypt argument, need file name with data")
        exit(-1)
    else:
        file_name = sys.argv[1]

    dis = Display(file_name)
    dis.main_loop()


if __name__ == '__main__':
    main()
