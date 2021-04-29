#!/usr/bin/python3

import json
import sys
import enum
import math
import numpy as np
import heapq
import random
import csv
import time
import xml.etree.ElementTree as ET

from casadi import *
import do_mpc

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

from testBot_robot import ZoneColor
from testBot_robot import PuckColor
from testBot_robot import OrientationFromTag

import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as colors


def decode_command(sentence):
    command = json.loads(sentence)
    return int(command['robot']), command['name'], command['values']

def code_command(robot_number, command_name, paramList):
    value = []
    for item in paramList:
        value.append(item)
    code = json.dumps({"robot": robot_number, "name": command_name, "values": value})
    return code

class RobotCommand(enum.Enum):
    ENTER_TO_ZONE = 'enterToZone'
    TAKE_PUCK = 'takeNearestPuck'
    ESCAPE_ZONE = 'escapeFromZone'
    PUT_DOWN_PUCK = 'putDownPuckOnPosition'
    GET_PUCK_COLOR = 'getPuckColor'
    GET_ORIENTATION = 'getOrientation'
    MOVE_TO_POSITION = 'moveToPosition'
    MOVE_WITH_ORIENTATION = 'moveWithOrientation'

    def __str__(self):
        return str(self.value)

class RobotState(enum.IntEnum):
    IDLE = 0
    FORWARD = 1
    TAKE_PUCK = 4
    ESCAPE_ZONE = 5
    WAIT = 6
    PUT_DOWN_PUCK = 7
    READY = 8
    ENTER_ZONE = 9
    FINISH = 10

class RobotParameters:
    def __init__(self, robotName, position, orientation, finishPosition, callbackRobotAnswer):
        self.robotName = robotName
        self.robotState = RobotState.IDLE
        self.robotPosition = position
        self.puckColor = PuckColor.NONPUCK
        self.robotInZone = False
        self.zoneColor = ZoneColor.BLACK
        self.zoneNumber = 0
        self.globalOrientation = orientation
        self.finishPosition = finishPosition
        self.backToFinishPosition = False
        self.aimPosition = position
        self.newPathStep = False
        self.canMakeNewStep = False
        self.nextStep = position
        self.pathLength = 0
        self.totalPathLength = 0
        self.stopCounter = 0
        self.packs_delivered = 0

        self.pubController = rospy.Publisher('/' + robotName + '/robotController', String, queue_size=10)
        self.pubRobotStop = rospy.Publisher("/" + robotName + "/robotStop", Bool, queue_size=10)
        rospy.Subscriber("/" + robotName + "/robotAnswer", String, callbackRobotAnswer)

    def sendCommand(self, robotNumber, commandName : String, paramList : list):
        self.pubController.publish(code_command(robotNumber, commandName, paramList))

class Controller:
    def __init__(self, file_robots, file_csv):

        self.puckInZone = dict()
        self.zoneEnterPosition = dict()
        self.listZoneEnterPosition = list()
        self.mpc = 0
        self.u0 = 0

        self.bots = []
        self.botsFinished = []
        self.file_robots_name = file_robots

        self.planning_cycle_counter = 0

        if file_robots == '':
            self.bots.append(RobotParameters('bot1', (3, 6), -math.pi/2, (3, 6), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot2',  (5, 3), math.pi, (5, 3), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot3', (8, 5), math.pi/2, (8, 5), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot4', (6, 8), 0, (6, 8), self.callbackRobotAnswerBot))
            '''
            self.bots.append(RobotParameters('bot5', (2, 5), -math.pi/2, (2, 5), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot6', (6, 2), math.pi, (6, 2), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot7', (9, 6), math.pi/2, (9, 6), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot8', (5, 9), 0, (5, 9), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot9', (0, 7), -math.pi/2, (0, 7), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot10', (4, 0), math.pi, (4, 0), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot11', (11, 4), math.pi/2, (11, 4), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot12', (7, 11), 0, (7, 11), self.callbackRobotAnswerBot))
            '''
            self.botsFinished = [False, False, False, False]#, False, False, False, False, False, False, False, False]

        else:
            self.readRobotsFromFile()

        self.SetEnterZonePositionMPC()
        self.SetPuckInZone()

        self.modelType = 1
        if self.modelType == 2:
            model = self.robotKinematicModel2()
            self.setupMPC2(model)

        else:
            model = self.robotKinematicModel()
            self.setupMPC(model)

        self.GenerateStartMap()
        #self.GenerateStartPathMap()

        #self.pathWasChange = True

        if file_csv == '':
            self.file_log_name = 'controller_log.csv'
        else:
            self.file_log_name = file_csv

        self.file_log_csv = open(self.file_log_name, 'w')
        self.csv_writer = csv.writer(self.file_log_csv, delimiter=';')
        self.csv_writer.writerow([len(self.bots), 'p'])

        print()

        '''
        plt.ion()
        self.fig = plt.figure()

        plt.xticks(np.arange(0, 3, 0.2))
        plt.yticks(np.arange(0, 3, 0.2))
        plt.grid(True)

        plt.show()
        '''
        print("Start")
        rospy.sleep(1)

        self.start_time = time.time()

    def __del__(self):
        self.file_log_csv.close()

    def readRobotsFromFile(self):

        tree = ET.parse(self.file_robots_name)
        root = tree.getroot()

        for arena in root.findall('arena'):
            for bot in arena.findall('foot-bot'):
                id = bot.get('id')
                body = bot.find('body')
                position = body.get('position').split(',')
                position = self.ConvertOnMapCoordinates((float(position[0]), float(position[1])))
                orientation = float(body.get('orientation').split(',')[0])
                if orientation <= -180:
                    orientation = 2 * math.pi * (orientation + 360) / 360
                else:
                    orientation = 2 * math.pi * orientation / 360

                self.bots.append(RobotParameters(id, position, orientation, position, self.callbackRobotAnswerBot))
                self.botsFinished.append(False)

    def mianLoop(self):

        for i in range(len(self.bots)):
            self.updateRobot(i+1)

        self.CalculationOnePathStep()
        #self.DrawCostFunction()

        finished = True
        for i in self.botsFinished:
            if not i:
                finished = False
                break

        if finished:
            self.file_log_csv.write('#finish')
            print("Work finished !!!")
            exit(0)

        '''
        if self.pathWasChange:
            sumMap = []
            k = 0
            for i in self.robotPathMap:
                sumMap.append([])
                for j in i:
                    if len(j) > 0:
                        sumMap[k].append(sum(j))
                    else:
                        sumMap[k].append(math.inf)
                k+=1
            plt.clf()
            ax1 = self.fig.add_subplot(221)
            ax2 = self.fig.add_subplot(222)
            ax3 = self.fig.add_subplot(223)
            ax4 = self.fig.add_subplot(224)
            self.fig.tight_layout(pad=3.0)
            ax1.set_title("Path map")
            ax2.set_title("Robot position map")
            ax3.set_title("Cost map bot2")
            ax4.set_title("Cost map bot3")
            ax1.set_xticks(np.arange(0, 12, 1))
            ax1.set_yticks(np.arange(0, 12, 1))
            ax2.set_xticks(np.arange(0, 12, 1))
            ax2.set_yticks(np.arange(0, 12, 1))
            ax3.set_xticks(np.arange(0, 12, 1))
            ax3.set_yticks(np.arange(0, 12, 1))
            ax4.set_xticks(np.arange(0, 12, 1))
            ax4.set_yticks(np.arange(0, 12, 1))
            ax1.grid(True)
            ax2.grid(True)
            ax3.grid(True)
            ax4.grid(True)
            ax1.imshow(np.array(sumMap).transpose(), cmap=cm.RdYlGn, origin='upper', extent=[0, 11, 0, 11])
            ax2.imshow(np.array(self.robotMap).transpose(), cmap=cm.RdYlGn, origin='upper', extent=[0, 11, 0, 11])
            ax3.imshow(np.array(self.GenerateCostMapForRobot(2)).transpose(), cmap=cm.RdYlGn, origin='upper', extent=[0, 11, 0, 11])
            ax4.imshow(np.array(self.GenerateCostMapForRobot(3)).transpose(), cmap=cm.RdYlGn, origin='upper',
                       extent=[0, 11, 0, 11])
            plt.pause(0.2)
            self.pathWasChange = False
        '''

    def updateRobot(self, robotNumber):
        robotState = self.bots[robotNumber-1].robotState

        if robotState == RobotState.FINISH:
            if self.botsFinished[robotNumber-1]:
                return

            elif self.bots[robotNumber-1].robotPosition == self.bots[robotNumber-1].finishPosition:
                self.botsFinished[robotNumber - 1] = True

            elif self.bots[robotNumber-1].robotPosition != self.bots[robotNumber-1].finishPosition:
                self.bots[robotNumber - 1].aimPosition = self.bots[robotNumber-1].finishPosition
                self.bots[robotNumber-1].robotState = RobotState.READY

        elif robotState == RobotState.IDLE:
            if self.bots[robotNumber-1].puckColor == PuckColor.NONPUCK:
                self.SearchNearestZone(ZoneColor.BLACK, robotNumber)
                self.bots[robotNumber - 1].zoneColor = ZoneColor.BLACK

                self.bots[robotNumber - 1].robotState = RobotState.READY

        elif robotState == RobotState.ENTER_ZONE and self.bots[robotNumber-1].robotInZone:
            if self.bots[robotNumber-1].puckColor == PuckColor.NONPUCK:
                self.bots[robotNumber - 1].robotState = RobotState.TAKE_PUCK
                self.bots[robotNumber - 1].sendCommand(robotNumber, str(RobotCommand.TAKE_PUCK), [])

            else:
                self.bots[robotNumber - 1].robotState = RobotState.PUT_DOWN_PUCK
                x, y = self.calcPuckPositionInZone(self.bots[robotNumber - 1].zoneColor, self.bots[robotNumber - 1].zoneNumber)
                self.bots[robotNumber-1].sendCommand(robotNumber, str(RobotCommand.PUT_DOWN_PUCK), [x, y])

        elif robotState == RobotState.READY:
            self.bots[robotNumber - 1].canMakeNewStep = True

            if self.bots[robotNumber - 1].aimPosition != self.bots[robotNumber - 1].robotPosition and \
                    self.bots[robotNumber - 1].newPathStep:

                self.MakeOneStep(robotNumber)
                self.bots[robotNumber - 1].canMakeNewStep = False
                self.bots[robotNumber - 1].newPathStep = False

            elif self.bots[robotNumber - 1].aimPosition == self.bots[robotNumber - 1].robotPosition:

                if not self.bots[robotNumber - 1].backToFinishPosition:
                    self.bots[robotNumber - 1].canMakeNewStep = False
                    self.bots[robotNumber - 1].robotState = RobotState.ENTER_ZONE
                    self.bots[robotNumber - 1].sendCommand(robotNumber, str(RobotCommand.ENTER_TO_ZONE),
                                                           [self.bots[robotNumber - 1].zoneColor])

                elif self.bots[robotNumber - 1].backToFinishPosition:
                    self.bots[robotNumber - 1].canMakeNewStep = False
                    self.bots[robotNumber - 1].robotState = RobotState.FINISH

    def callbackRobotAnswerBot(self, msg):
        robotNumber, answer, answerParam = decode_command(msg.data)
        print("Robot: {}, answer {}, parma: {}".format(self.bots[robotNumber-1].robotName, answer, answerParam))


        if answer == str(RobotCommand.MOVE_TO_POSITION):
            if len(answerParam) == 3:
                self.bots[robotNumber-1].globalOrientation = answerParam[2]

                if self.bots[robotNumber - 1].robotPosition != self.bots[robotNumber - 1].nextStep:
                    rob_pos = self.bots[robotNumber - 1].robotPosition
                    self.robotMap[rob_pos[0]][rob_pos[1]] = 0
                    conv_pos = self.ConvertOnRealCoordinates(rob_pos[0], rob_pos[1])
                    conv_next_pos = self.ConvertOnRealCoordinates(self.bots[robotNumber-1].nextStep[0], self.bots[robotNumber-1].nextStep[1])
                    self.bots[robotNumber - 1].pathLength += self.Norm2(conv_pos, conv_next_pos)
                    self.bots[robotNumber - 1].totalPathLength += self.TaxicabNorm(rob_pos, self.bots[robotNumber-1].nextStep)
                    self.bots[robotNumber-1].robotPosition = self.bots[robotNumber-1].nextStep

                self.bots[robotNumber-1].robotState = RobotState.READY

        elif answer == str(RobotCommand.MOVE_WITH_ORIENTATION):
            if len(answerParam) == 3:
                self.bots[robotNumber-1].globalOrientation = answerParam[2]

                if self.bots[robotNumber - 1].robotPosition != self.bots[robotNumber - 1].nextStep:
                    rob_pos = self.bots[robotNumber - 1].robotPosition
                    self.robotMap[rob_pos[0]][rob_pos[1]] = 0
                    conv_pos = self.ConvertOnRealCoordinates(rob_pos[0], rob_pos[1])
                    conv_next_pos = self.ConvertOnRealCoordinates(self.bots[robotNumber - 1].nextStep[0], self.bots[robotNumber - 1].nextStep[1])
                    self.bots[robotNumber - 1].pathLength += self.Norm2(conv_pos, conv_next_pos)
                    self.bots[robotNumber - 1].totalPathLength += self.TaxicabNorm(rob_pos, self.bots[robotNumber-1].nextStep)
                    self.bots[robotNumber - 1].robotPosition = self.bots[robotNumber - 1].nextStep

                self.bots[robotNumber-1].robotState = RobotState.READY

        elif answer == str(RobotCommand.ENTER_TO_ZONE):
            if len(answerParam) == 1:
                if answerParam[0] != -1:
                    self.bots[robotNumber-1].robotInZone = True
                    self.bots[robotNumber-1].zoneNumber = answerParam[0]

        elif answer == str(RobotCommand.TAKE_PUCK):
            if len(answerParam) == 1:
                if answerParam[0] == PuckColor.NONPUCK:
                    self.puckInZone[self.bots[robotNumber-1].zoneColor][self.bots[robotNumber-1].zoneNumber] = 0

                    for i in range(len(self.bots)):
                        if i == robotNumber - 1:
                            continue
                        else:
                            if self.bots[i].aimPosition == self.zoneEnterPosition[self.bots[robotNumber-1].zoneColor][self.bots[robotNumber-1].zoneNumber]:
                                self.SearchNearestZone(self.bots[robotNumber-1].zoneColor, i+1)

                self.bots[robotNumber-1].puckColor = answerParam[0]

                self.bots[robotNumber-1].robotState = RobotState.ESCAPE_ZONE
                self.bots[robotNumber-1].sendCommand(robotNumber, str(RobotCommand.ESCAPE_ZONE), [])

        elif answer == str(RobotCommand.ESCAPE_ZONE):
            if len(answerParam) == 3:
                self.bots[robotNumber-1].globalOrientation = answerParam[2]
                self.bots[robotNumber - 1].robotPosition = self.zoneEnterPosition[self.bots[robotNumber - 1].zoneColor][self.bots[robotNumber - 1].zoneNumber]

                self.bots[robotNumber-1].robotInZone = False
                if self.bots[robotNumber-1].puckColor == PuckColor.NONPUCK:
                    self.bots[robotNumber-1].zoneColor = ZoneColor.BLACK

                else:
                    puckColor = self.bots[robotNumber-1].puckColor
                    if puckColor == PuckColor.RED:
                        self.bots[robotNumber-1].zoneColor = ZoneColor.RED

                    elif puckColor == PuckColor.GREEN:
                        self.bots[robotNumber-1].zoneColor = ZoneColor.GREEN

                    elif puckColor == PuckColor.YELLOW:
                        self.bots[robotNumber-1].zoneColor = ZoneColor.YELLOW

                    elif puckColor == PuckColor.BLUE:
                        self.bots[robotNumber-1].zoneColor = ZoneColor.BLUE

                self.SearchNearestZone(self.bots[robotNumber-1].zoneColor, robotNumber)
                self.bots[robotNumber-1].robotState = RobotState.READY


        elif answer == str(RobotCommand.PUT_DOWN_PUCK):
            if len(answerParam) == 1:
                if answerParam[0]:
                    self.bots[robotNumber - 1].packs_delivered += 1

                    self.puckInZone[self.bots[robotNumber-1].zoneColor][self.bots[robotNumber-1].zoneNumber] += 1
                    self.bots[robotNumber-1].puckColor = PuckColor.NONPUCK

                    self.bots[robotNumber-1].robotState = RobotState.ESCAPE_ZONE
                    self.bots[robotNumber-1].sendCommand(robotNumber, str(RobotCommand.ESCAPE_ZONE), [])

        elif answer == str(RobotCommand.GET_ORIENTATION):
            if len(answerParam) == 1:
                self.bots[robotNumber-1].globalOrientation = answerParam[0]

        elif answer == str(RobotCommand.GET_PUCK_COLOR):
            if len(answerParam) == 1:
                self.bots[robotNumber-1].puckColor = answerParam[0]

    def calcPuckPositionInZone(self, zoneColor: ZoneColor, zoneNumber):
        puckNumberInZone = self.puckInZone[zoneColor][zoneNumber]
        xPos = 0
        yPos = 0

        if zoneColor == ZoneColor.RED:
            if zoneNumber == 1:
                if puckNumberInZone <= 24:
                    if puckNumberInZone <= 7:
                        y = puckNumberInZone % 4
                        x = math.floor(puckNumberInZone / 4.0)
                        yPos = 2.325 - y * 0.15
                        xPos = -2.9 + x * 0.15

                    elif puckNumberInZone >= 8 and puckNumberInZone <= 15:
                        y = puckNumberInZone % 4
                        x = math.floor(puckNumberInZone / 4.0) - 2
                        yPos = 2.325 - y * 0.15
                        xPos = -1.75 - x * 0.15

                    else:
                        x = puckNumberInZone % 4
                        y = math.floor(puckNumberInZone / 4.0) - 8
                        yPos = 2.325 - y * 0.15
                        xPos = -2.65 + x * 0.15

            if zoneNumber == 2:
                if puckNumberInZone <= 24:
                    y = puckNumberInZone % 4
                    x = math.floor(puckNumberInZone/4.0)
                    yPos = 2.475 + y*0.15
                    xPos = -2.9 + x*0.15

        elif zoneColor == ZoneColor.GREEN:
            if zoneNumber == 1:
                if puckNumberInZone <= 24:
                    if puckNumberInZone <= 7:
                        y = puckNumberInZone % 4
                        x = math.floor(puckNumberInZone / 4.0)
                        yPos = -2.325 + y * 0.15
                        xPos = -2.9 + x * 0.15

                    elif puckNumberInZone >= 8 and puckNumberInZone <= 15:
                        y = puckNumberInZone % 4
                        x = math.floor(puckNumberInZone / 4.0) - 2
                        yPos = -2.325 + y * 0.15
                        xPos = -1.75 - x * 0.15

                    else:
                        x = puckNumberInZone % 4
                        y = math.floor(puckNumberInZone / 4.0) - 8
                        yPos = -2.325 + y * 0.15
                        xPos = -2.65 + x * 0.15

            if zoneNumber == 2:
                if puckNumberInZone <= 24:
                    y = puckNumberInZone % 4
                    x = math.floor(puckNumberInZone / 4.0)
                    yPos = -2.475 - y * 0.15
                    xPos = -2.9 + x * 0.15

        elif zoneColor == ZoneColor.YELLOW:
            if zoneNumber == 1:
                if puckNumberInZone <= 24:
                    if puckNumberInZone <= 7:
                        y = puckNumberInZone % 4
                        x = math.floor(puckNumberInZone / 4.0)
                        yPos = -2.325 + y * 0.15
                        xPos = 2.9 - x * 0.15

                    elif puckNumberInZone >= 8 and puckNumberInZone <= 15:
                        y = puckNumberInZone % 4
                        x = math.floor(puckNumberInZone / 4.0) - 2
                        yPos = -2.325 + y * 0.15
                        xPos = 1.75 + x * 0.15

                    else:
                        x = puckNumberInZone % 4
                        y = math.floor(puckNumberInZone / 4.0) - 8
                        yPos = -2.325 + y * 0.15
                        xPos = 2.65 - x * 0.15

            if zoneNumber == 2:
                if puckNumberInZone <= 24:
                    y = puckNumberInZone % 4
                    x = math.floor(puckNumberInZone / 4.0)
                    yPos = -2.475 - y * 0.15
                    xPos = 2.9 - x * 0.15


        elif zoneColor == ZoneColor.BLUE:
            if zoneNumber == 1:
                if puckNumberInZone <= 24:
                    if puckNumberInZone <= 7:
                        y = puckNumberInZone % 4
                        x = math.floor(puckNumberInZone / 4.0)
                        yPos = 2.325 - y * 0.15
                        xPos = 2.9 - x * 0.15

                    elif puckNumberInZone >= 8 and puckNumberInZone <= 15:
                        y = puckNumberInZone % 4
                        x = math.floor(puckNumberInZone / 4.0) - 2
                        yPos = 2.325 - y * 0.15
                        xPos = 1.75 + x * 0.15

                    else:
                        x = puckNumberInZone % 4
                        y = math.floor(puckNumberInZone / 4.0) - 8
                        yPos = 2.325 - y * 0.15
                        xPos = 2.65 - x * 0.15

            if zoneNumber == 2:
                if puckNumberInZone <= 24:
                    y = puckNumberInZone % 4
                    x = math.floor(puckNumberInZone/4.0)
                    yPos = 2.475 + y*0.15
                    xPos = 2.9 - x*0.15

        return xPos, yPos

    def tvp_fun1(self, t_now):

        temp = self.mpc.get_tvp_template()
        temp_list = list()
        for i in range(len(self.bots)):
            aim_conv = self.ConvertOnRealCoordinates(self.bots[i].aimPosition[0], self.bots[i].aimPosition[1])
            temp_list.append(aim_conv[0])

        temp['_tvp', :, 'aim_x'] = np.array(temp_list)
        temp_list = list()
        for i in range(len(self.bots)):
            aim_conv = self.ConvertOnRealCoordinates(self.bots[i].aimPosition[0], self.bots[i].aimPosition[1])
            temp_list.append(aim_conv[1])

        temp['_tvp', :, 'aim_y'] = np.array(temp_list)

        return temp

    def dist_to_line(self, x, y, A, B, C):
        return fabs(A * x + B * y + C) / (sqrt(A ** 2 + B ** 2))

    def dist_to_section(self, x, y, point1, point2):
        t = ((point2[0] - point1[0]) * (x - point1[0]) + (point2[1] - point1[1]) * (y - point1[1])) / \
            ((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)
        t = fmax(0, fmin(1, t))
        projection = (point1[0] + t * (point2[0] - point1[0]), point1[1] + t * (point2[1] - point1[1]))
        #dist = if_else(lt(fabs(x - projection[0]), 0.0001)*lt(fabs(y - projection[1]), 0.0001), 0.0001, self.Norm2([x, y], projection))
        dist = self.Norm2Plus([x, y], projection)
        return dist

    def functionGrid(self, x, y):
        temp_min = self.dist_to_section(x, y, [1.2, 2.7], [-1.2, 2.7]) #pionowa 2.7
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [1.2, 2.25], [-1.2, 2.25])) #pionowa 2.25
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [1.2, 1.8], [-1.2, 1.8])) #pionowa 1.8
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.85, 1.2], [-2.85, 1.2]))  # pionowa 1.2
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.85, 0.75], [0.3, 0.75]))  # pionowa gorna 0.75
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-0.75, 0.75], [-2.85, 0.75]))  # pionowa dolna 0.75
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.85, 0.3], [1.2, 0.3]))  # pionowa gorna 0.3
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-0.75, 0.3], [-2.85, 0.3]))  # pionowa dolna 0.3
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.85, -0.3], [0.75, -0.3]))  # pionowa gorna -0.3
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-1.2, -0.3], [-2.85, -0.3]))  # pionowa dolna -0.3
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.85, -0.75], [0.75, -0.75]))  # pionowa gorna -0.75
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-0.3, -0.75], [-2.85, -0.75]))  # pionowa dolna -0.75
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.85, -1.2], [-2.85, -1.2]))  # pionowa -1.2
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [1.2, -1.8], [-1.2, -1.8]))  # pionowa -1.8
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [1.2, -2.25], [-1.2, -2.25]))  # pionowa -2.25
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [1.2, -2.7], [-1.2, -2.7]))  # pionowa -2.7

        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.85, 1.2], [2.85, -1.2]))  # pozioma 2.7
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.4, 1.2], [2.4, -1.2]))  # pozioma 2.4
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [1.8, 1.2], [1.8, -1.2]))  # pozioma 1.8
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [1.2, 2.7], [1.2, -2.7]))  # pozioma 1.2
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [0.75, 2.7], [0.75, 0.75]))  # pozioma lewa 0.75
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [0.75, -0.3], [0.75, -2.7]))  # pozioma prawa 0.75
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [0.3, 2.7], [0.3, 0.75]))  # pozioma lewa 0.3
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [0.3, -1.2], [0.3, -2.7]))  # pozioma prawa 0.3
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-0.3, 2.7], [-0.3, 1.2]))  # pozioma lewa -0.3
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-0.3, -0.75], [-0.3, -2.7]))  # pozioma prawa -0.3
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-0.75, 2.7], [-0.75, 0.3]))  # pozioma lewa -0.75
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-0.75, -0.75], [-0.75, -2.7]))  # pozioma prawa -0.75
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-1.2, 2.7], [-1.2, -2.7]))  # pozioma -1.2
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-1.8, 1.2], [-1.8, -1.2]))  # pozioma -1.8
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-2.4, 1.2], [-2.4, -1.2]))  # pozioma -2.4
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-2.85, 1.2], [-2.85, -1.2]))  # pozioma -2.7

        return temp_min

    def functionGrid2(self, x, y):
        scale = 100
        shift = 0.05

        temp_min = self.SaturationFunctionGrid(self.dist_to_section(x, y, [1.2, 2.7], [-1.2, 2.7]), scale, shift)  # pionowa 2.7
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [1.2, 2.25], [-1.2, 2.25]), scale, shift)  # pionowa 2.25
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [1.2, 1.8], [-1.2, 1.8]), scale, shift)  # pionowa 1.8
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [2.85, 1.2], [-2.85, 1.2]), scale, shift)  # pionowa 1.2
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [2.85, 0.75], [0.3, 0.75]), scale, shift)  # pionowa gorna 0.75
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [-0.75, 0.75], [-2.85, 0.75]), scale, shift)  # pionowa dolna 0.75
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [2.85, 0.3], [1.2, 0.3]), scale, shift)  # pionowa gorna 0.3
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [-0.75, 0.3], [-2.85, 0.3]), scale, shift)  # pionowa dolna 0.3
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [2.85, -0.3], [0.75, -0.3]), scale, shift)  # pionowa gorna -0.3
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [-1.2, -0.3], [-2.85, -0.3]), scale, shift)  # pionowa dolna -0.3
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [2.85, -0.75], [0.75, -0.75]), scale, shift) # pionowa gorna -0.75
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [-0.3, -0.75], [-2.85, -0.75]), scale, shift) #pionowa dolna -0.75
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [2.85, -1.2], [-2.85, -1.2]), scale, shift)  # pionowa -1.2
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [1.2, -1.8], [-1.2, -1.8]), scale, shift)  # pionowa -1.8
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [1.2, -2.25], [-1.2, -2.25]), scale, shift)  # pionowa -2.25
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [1.2, -2.7], [-1.2, -2.7]), scale, shift)  # pionowa -2.7

        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [2.85, 1.2], [2.85, -1.2]), scale, shift)  # pozioma 2.7
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [2.4, 1.2], [2.4, -1.2]), scale, shift)  # pozioma 2.4
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [1.8, 1.2], [1.8, -1.2]), scale, shift)  # pozioma 1.8
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [1.2, 2.7], [1.2, -2.7]), scale, shift)  # pozioma 1.2
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [0.75, 2.7], [0.75, 0.75]), scale, shift)  # pozioma lewa 0.75
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [0.75, -0.3], [0.75, -2.7]), scale, shift)  # pozioma prawa 0.75
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [0.3, 2.7], [0.3, 0.75]), scale, shift)  # pozioma lewa 0.3
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [0.3, -1.2], [0.3, -2.7]), scale, shift)  # pozioma prawa 0.3
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [-0.3, 2.7], [-0.3, 1.2]), scale, shift)  # pozioma lewa -0.3
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [-0.3, -0.75], [-0.3, -2.7]), scale, shift)  # pozioma prawa -0.3
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [-0.75, 2.7], [-0.75, 0.3]), scale, shift)  # pozioma lewa -0.75
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [-0.75, -0.75], [-0.75, -2.7]), scale, shift)#pozioma prawa -0.75
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [-1.2, 2.7], [-1.2, -2.7]), scale, shift)  # pozioma -1.2
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [-1.8, 1.2], [-1.8, -1.2]), scale, shift)  # pozioma -1.8
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [-2.4, 1.2], [-2.4, -1.2]), scale, shift)  # pozioma -2.4
        temp_min *= self.SaturationFunctionGrid(self.dist_to_section(x, y, [-2.85, 1.2], [-2.85, -1.2]), scale, shift)  # pozioma -2.7

        return temp_min

    def functionAvoid(self, x, y, currentBot, length):
        if length == 1:
            return SX(0)
        else:
            temp = np.inf
            for i in range(length):
                if i == currentBot:
                    continue
                else:
                    temp = fmin(temp, self.Norm2((x[currentBot], y[currentBot]), (x[i], y[i])))

        return temp

    def robotKinematicModel(self):
        model_type = 'continuous'
        model = do_mpc.model.Model(model_type)
        pos_x = model.set_variable('_x', 'pos_x', shape=(len(self.bots), 1))
        pos_y = model.set_variable('_x', 'pos_y', shape=(len(self.bots), 1))
        theta = model.set_variable('_x', 'theta', shape=(len(self.bots), 1))
        distance = model.set_variable('_x', 'distance', shape=(len(self.bots), 1))

        v = model.set_variable('_u', 'v', shape=(len(self.bots), 1))
        omega = model.set_variable('_u', 'omega', shape=(len(self.bots), 1))

        aim_x = model.set_variable('_tvp', 'aim_x', shape=(len(self.bots), 1))
        aim_y = model.set_variable('_tvp', 'aim_y', shape=(len(self.bots), 1))

        # model.set_rhs('pos_x', v * cos(omega))
        # model.set_rhs('pos_y', v * sin(omega))
        dpos_x = list()
        dpos_y = list()
        dtheta = list()
        ddistance = list()
        nl_cons_grid = list()
        nl_cons_avoid = list()

        for i in range(len(self.bots)):
            dpos_x.append(v[i]*cos(theta[i]))
            dpos_y.append(v[i]*sin(theta[i]))
            dtheta.append(omega[i])
            ddistance.append(v[i])

            nl_cons_grid.append(self.functionGrid2(pos_x[i], pos_y[i]))
            nl_cons_avoid.append(self.functionAvoid(pos_x, pos_y, i, len(self.bots)))

        model.set_rhs('pos_x', vertcat(*dpos_x))
        model.set_rhs('pos_y', vertcat(*dpos_y))
        model.set_rhs('theta', vertcat(*dtheta))
        model.set_rhs('distance', vertcat(*ddistance))

        model.set_expression('grid', vertcat(*nl_cons_grid))
        model.set_expression('avoid', vertcat(*nl_cons_avoid))

        model.setup()

        return model

    def setupMPC(self, model):

        setup_mpc = {
            'n_horizon': 15,
            't_step': 0.5,
            'n_robust': 0,
            'store_full_solution': True,
            'nlpsol_opts': {'ipopt.max_iter': 1500, 'ipopt.print_level': 3, 'ipopt.linear_solver': 'ma27'}#, 'ipopt.tol': 10e-2, 'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
            # 'ipopt.max_iter':500
        }

        self.mpc = do_mpc.controller.MPC(model)
        self.mpc.set_param(**setup_mpc)
        self.mpc.set_tvp_fun(self.tvp_fun1)

        w_lterm_dist = 55
        w_lterm_path = 19
        w_lterm_avoid = 15
        w_mterm_dist = 80

        w_mterm = 6
        w_lterm = 1

        mterm = SX(0)  # 10*fabs(model.x['pos_x'] - model.tvp['aim_x']) + 10*fabs(model.x['pos_y'] - model.tvp['aim_y'])
        lterm = SX(0)

        for i in range(len(self.bots)):
            mterm += w_mterm_dist * self.MyNorm2((model.x['pos_x', i], model.x['pos_y', i]), (model.tvp['aim_x', i], model.tvp['aim_y', i]))
            #mterm += w_mterm_dist*fabs(model.x['pos_x', i] - model.tvp['aim_x', i]) + w_mterm_dist*fabs(model.x['pos_y', i] - model.tvp['aim_y', i])
            #lterm += w_lterm_dist*fabs(model.x['pos_x', i] - model.tvp['aim_x', i]) \
            #          + w_lterm_dist*fabs(model.x['pos_y', i] - model.tvp['aim_y', i])
            lterm += w_lterm_dist * self.MyNorm2((model.x['pos_x', i], model.x['pos_y', i]), (model.tvp['aim_x', i], model.tvp['aim_y', i]))
            lterm += w_lterm_path*model.x['distance', i]


        for i in range(len(self.bots)):
            for j in range(len(self.bots)):
                if i == j:
                    continue
                else:
                    pass
                    lterm += w_lterm_avoid*self.SaturationFunction(fabs(model.x['pos_x', i] - model.x['pos_x', j]), 2)\
                             + w_lterm_avoid*self.SaturationFunction(fabs(model.x['pos_y', i] - model.x['pos_y', j]), 2)


        #lterm /= (2*len(self.bots)*w_lterm_dist)+(2*(len(self.bots)-1)*len(self.bots)*w_lterm_avoid)
        #mterm /= 2*len(self.bots)*w_mterm_dist
        lterm /= setup_mpc['n_horizon']
        lterm *= 0.5

        #lterm *= w_lterm / (w_lterm + w_mterm)
        #mterm *= w_mterm / (w_lterm + w_mterm)

        rterm_v = list()
        rterm_omega = list()

        for i in range(len(self.bots)):
            rterm_v.append([0])
            rterm_omega.append([0])

        self.mpc.set_objective(mterm=mterm, lterm=lterm)
        self.mpc.set_rterm(v=np.array(rterm_v), omega=np.array(rterm_omega))

        min_x = list()
        min_y = list()
        min_theta = list()

        max_x = list()
        max_y = list()
        max_theta = list()

        min_v = list()
        min_omega = list()

        max_v = list()
        max_omega = list()

        for i in range(len(self.bots)):
            min_x.append([-3.0])
            min_y.append([-3.0])
            min_theta.append([-np.pi])

            max_x.append([3.0])
            max_y.append([3.0])
            max_theta.append([np.pi])

            min_v.append([0])
            min_omega.append([-2])

            max_v.append([0.4])
            max_omega.append([2])


        self.mpc.bounds['lower', '_x', 'pos_x'] = np.array(min_x)
        self.mpc.bounds['lower', '_x', 'pos_y'] = np.array(min_y)
        self.mpc.bounds['lower', '_x', 'theta'] = np.array(min_theta)

        self.mpc.bounds['upper', '_x', 'pos_x'] = np.array(max_x)
        self.mpc.bounds['upper', '_x', 'pos_y'] = np.array(max_y)
        self.mpc.bounds['upper', '_x', 'theta'] = np.array(max_theta)

        self.mpc.bounds['lower', '_u', 'v'] = np.array(min_v)
        self.mpc.bounds['lower', '_u', 'omega'] = np.array(min_omega)
        #bot.mpc.bounds['lower', '_u', 'omega'] = -np.pi

        self.mpc.bounds['upper', '_u', 'v'] = np.array(max_v)
        self.mpc.bounds['upper', '_u', 'omega'] = np.array(max_omega)
        #bot.mpc.bounds['lower', '_u', 'omega'] = np.pi

        nl_cons_value_grid = list()
        for i in range(len(self.bots)):
            nl_cons_value_grid.append([0.02])

        nl_cons_value_avoid = list()
        for i in range(len(self.bots)):
            nl_cons_value_avoid.append([-0.41])

        self.mpc.set_nl_cons('grid_shape', model.aux['grid'], np.array(nl_cons_value_grid))
        self.mpc.set_nl_cons('avoid_robots', -model.aux['avoid'], np.array(nl_cons_value_avoid))

        self.mpc.setup()

        x0 = list()
        for i in range(len(self.bots)):
            conv_pos = self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1])
            x0.append(conv_pos[0])

        for i in range(len(self.bots)):
            conv_pos = self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1])
            x0.append(conv_pos[1])

        for i in range(len(self.bots)):
            x0.append(self.bots[i].globalOrientation)

        for i in range(len(self.bots)):
            x0.append(self.bots[i].pathLength)

        x0 = np.array(x0).reshape(-1, 1)
        self.mpc.x0 = x0
        self.mpc.set_initial_guess()

    def robotKinematicModel2(self):
        model_type = 'continuous'
        model = do_mpc.model.Model(model_type)
        pos_x = model.set_variable('_x', 'pos_x', shape=(len(self.bots), 1))
        pos_y = model.set_variable('_x', 'pos_y', shape=(len(self.bots), 1))
        distance = model.set_variable('_x', 'distance', shape=(len(self.bots), 1))

        v = model.set_variable('_u', 'v', shape=(len(self.bots), 1))
        omega = model.set_variable('_u', 'omega', shape=(len(self.bots), 1))

        aim_x = model.set_variable('_tvp', 'aim_x', shape=(len(self.bots), 1))
        aim_y = model.set_variable('_tvp', 'aim_y', shape=(len(self.bots), 1))

        dpos_x = list()
        dpos_y = list()
        ddistance = list()
        nl_cons_grid = list()
        nl_cons_avoid = list()

        for i in range(len(self.bots)):
            dpos_x.append(v[i]*cos(omega[i]))
            dpos_y.append(v[i]*sin(omega[i]))
            ddistance.append(v[i])

            nl_cons_grid.append(self.functionGrid2(pos_x[i], pos_y[i]))
            nl_cons_avoid.append(self.functionAvoid(pos_x, pos_y, i, len(self.bots)))

        model.set_rhs('pos_x', vertcat(*dpos_x))
        model.set_rhs('pos_y', vertcat(*dpos_y))
        model.set_rhs('distance', vertcat(*ddistance))

        model.set_expression('grid', vertcat(*nl_cons_grid))
        model.set_expression('avoid', vertcat(*nl_cons_avoid))

        model.setup()

        return model

    def setupMPC2(self, model):

        setup_mpc = {
            'n_horizon': 25,
            't_step': 0.5,
            'n_robust': 0,
            'store_full_solution': True,
            'nlpsol_opts': {'ipopt.max_iter': 1500, 'ipopt.print_level': 3, 'ipopt.linear_solver': 'ma27'}#, 'ipopt.tol': 10e-2, 'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
            # 'ipopt.max_iter':500
        }

        self.mpc = do_mpc.controller.MPC(model)
        self.mpc.set_param(**setup_mpc)
        self.mpc.set_tvp_fun(self.tvp_fun1)

        w_lterm_dist = 55
        w_lterm_path = 19
        w_lterm_avoid = 15
        w_mterm_dist = 80
        w_mterm = 6
        w_lterm = 1

        mterm = SX(0)  # 10*fabs(model.x['pos_x'] - model.tvp['aim_x']) + 10*fabs(model.x['pos_y'] - model.tvp['aim_y'])
        lterm = SX(0)

        for i in range(len(self.bots)):
            mterm += w_mterm_dist*fabs(model.x['pos_x', i] - model.tvp['aim_x', i]) + w_mterm_dist*fabs(model.x['pos_y', i] - model.tvp['aim_y', i])
            lterm += w_lterm_dist*fabs(model.x['pos_x', i] - model.tvp['aim_x', i]) \
                     + w_lterm_dist*fabs(model.x['pos_y', i] - model.tvp['aim_y', i])

            lterm += w_lterm_path * model.x['distance', i]

        for i in range(len(self.bots)):
            for j in range(len(self.bots)):
                if i == j:
                    continue
                else:
                    pass
                    lterm += w_lterm_avoid*self.SaturationFunction(fabs(model.x['pos_x', i] - model.x['pos_x', j]), 2)\
                             + w_lterm_avoid*self.SaturationFunction(fabs(model.x['pos_y', i] - model.x['pos_y', j]), 2)
                    #lterm += -15*sqrt((model.x['pos_x', i] - model.x['pos_x', j])**2 + (model.x['pos_y', i] - model.x['pos_y', j])**2)


        #lterm /= (2*len(self.bots)*w_lterm_dist)+(2*(len(self.bots)-1)*len(self.bots)*w_lterm_avoid)
        #mterm /= 2*len(self.bots)*w_mterm_dist
        lterm /= setup_mpc['n_horizon']
        lterm *= 0.5

        #lterm *= w_lterm / (w_lterm + w_mterm)
        #mterm *= w_mterm / (w_lterm + w_mterm)

        rterm_v = list()
        rterm_omega = list()

        for i in range(len(self.bots)):
            rterm_v.append([0])
            rterm_omega.append([0])

        self.mpc.set_objective(mterm=mterm, lterm=lterm)
        self.mpc.set_rterm(v=np.array(rterm_v), omega=np.array(rterm_omega))

        min_x = list()
        min_y = list()

        max_x = list()
        max_y = list()

        min_v = list()
        min_omega = list()

        max_v = list()
        max_omega = list()

        for i in range(len(self.bots)):
            min_x.append([-3.0])
            min_y.append([-3.0])

            max_x.append([3.0])
            max_y.append([3.0])

            min_v.append([0])
            min_omega.append([-np.pi])

            max_v.append([0.4])
            max_omega.append([np.pi])


        self.mpc.bounds['lower', '_x', 'pos_x'] = np.array(min_x)
        self.mpc.bounds['lower', '_x', 'pos_y'] = np.array(min_y)

        self.mpc.bounds['upper', '_x', 'pos_x'] = np.array(max_x)
        self.mpc.bounds['upper', '_x', 'pos_y'] = np.array(max_y)

        self.mpc.bounds['lower', '_u', 'v'] = np.array(min_v)
        self.mpc.bounds['lower', '_u', 'omega'] = np.array(min_omega)

        self.mpc.bounds['upper', '_u', 'v'] = np.array(max_v)
        self.mpc.bounds['upper', '_u', 'omega'] = np.array(max_omega)

        nl_cons_value_grid = list()
        for i in range(len(self.bots)):
            nl_cons_value_grid.append([0.02])

        nl_cons_value_avoid = list()
        for i in range(len(self.bots)):
            nl_cons_value_avoid.append([-0.41])

        self.mpc.set_nl_cons('grid_shape', model.aux['grid'], np.array(nl_cons_value_grid))
        self.mpc.set_nl_cons('avoid_robots', -model.aux['avoid'], np.array(nl_cons_value_avoid))

        self.mpc.setup()

        x0 = list()
        for i in range(len(self.bots)):
            conv_pos = self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1])
            x0.append(conv_pos[0])

        for i in range(len(self.bots)):
            conv_pos = self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1])
            x0.append(conv_pos[1])

        for i in range(len(self.bots)):
            x0.append(self.bots[i].pathLength)

        x0 = np.array(x0).reshape(-1, 1)
        self.mpc.x0 = x0
        self.mpc.set_initial_guess()

    def GenerateStartMap(self):
        self.robotMap = []
        for i in range(12):
            self.robotMap.append([])
            for j in range(12):
                self.robotMap[i].append(0)

        self.robotMap[0][0] = -1
        self.robotMap[1][0] = -1
        self.robotMap[2][0] = -1
        self.robotMap[0][1] = -1
        self.robotMap[1][1] = -1
        self.robotMap[1][2] = -1
        self.robotMap[2][2] = -1
        self.robotMap[9][0] = -1
        self.robotMap[10][0] = -1
        self.robotMap[11][0] = -1
        self.robotMap[10][1] = -1
        self.robotMap[11][1] = -1
        self.robotMap[9][2] = -1
        self.robotMap[10][2] = -1
        self.robotMap[5][5] = -1
        self.robotMap[6][5] = -1
        self.robotMap[5][6] = -1
        self.robotMap[6][6] = -1
        self.robotMap[1][9] = -1
        self.robotMap[2][9] = -1
        self.robotMap[0][10] = -1
        self.robotMap[1][10] = -1
        self.robotMap[0][11] = -1
        self.robotMap[1][11] = -1
        self.robotMap[2][11] = -1
        self.robotMap[9][9] = -1
        self.robotMap[10][9] = -1
        self.robotMap[10][10] = -1
        self.robotMap[11][10] = -1
        self.robotMap[9][11] = -1
        self.robotMap[10][11] = -1
        self.robotMap[11][11] = -1

        #MPC
        self.robotMap[4][6] = -1
        self.robotMap[5][4] = -1
        self.robotMap[7][5] = -1
        self.robotMap[6][7] = -1
        self.robotMap[0][9] = -1
        self.robotMap[2][10] = -1
        self.robotMap[9][10] = -1
        self.robotMap[11][9] = -1
        self.robotMap[11][2] = -1
        self.robotMap[9][1] = -1
        self.robotMap[2][1] = -1
        self.robotMap[0][2] = -1

        for i in range(len(self.bots)):
            self.robotMap[self.bots[i].robotPosition[0]][self.bots[i].robotPosition[1]] = i+1

        '''
        j = 1
        for i in self.bots:
            self.robotMap[i.robotPosition[0]][i.robotPosition[1]] = j
            if i.robotPosition in self.listZoneEnterPosition:
                neighbors = self.GetNeighbors(i.robotPosition)
                
                for k in neighbors:
                    self.robotMap[k[0]][k[1]] = j
            j = j + 1
        '''

    def GenerateStartPathMap(self):
        self.robotPathMap = []
        for i in range(12):
            self.robotPathMap.append([])
            for j in range(12):
                self.robotPathMap[i].append([])

    def AddRobotPathToMap(self, robotNumber):
        self.pathWasChange = True
        path = self.bots[robotNumber-1].path
        if len(path) > 0:
            for i in path:
                if robotNumber in self.robotPathMap[i[0]][i[1]]:
                    continue
                else:
                    self.robotPathMap[i[0]][i[1]].append(robotNumber)

    def RemoveOneStepFromPathMap(self, robotNumber, step):
        self.pathWasChange = True
        if robotNumber in self.robotPathMap[step[0]][step[1]]:
            self.robotPathMap[step[0]][step[1]].remove(robotNumber)

    def RemoveRobotPathFromPathMap(self, robotNumber):
        self.pathWasChange = True
        for i in self.bots[robotNumber-1].path:
            if robotNumber in self.robotPathMap[i[0]][i[1]]:
                self.robotPathMap[i[0]][i[1]].remove(robotNumber)

    def SetEnterZonePosition(self):
        self.zoneEnterPosition[ZoneColor.BLACK] = dict()
        self.zoneEnterPosition[ZoneColor.BLACK][1] = (6, 7)
        self.zoneEnterPosition[ZoneColor.BLACK][2] = (4, 6)
        self.zoneEnterPosition[ZoneColor.BLACK][3] = (5, 4)
        self.zoneEnterPosition[ZoneColor.BLACK][4] = (7, 5)

        self.zoneEnterPosition[ZoneColor.RED] = dict()
        self.zoneEnterPosition[ZoneColor.RED][1] = (2, 10)
        self.zoneEnterPosition[ZoneColor.RED][2] = (0, 9)

        self.zoneEnterPosition[ZoneColor.GREEN] = dict()
        self.zoneEnterPosition[ZoneColor.GREEN][1] = (9, 10)
        self.zoneEnterPosition[ZoneColor.GREEN][2] = (11, 9)

        self.zoneEnterPosition[ZoneColor.YELLOW] = dict()
        self.zoneEnterPosition[ZoneColor.YELLOW][1] = (9, 1)
        self.zoneEnterPosition[ZoneColor.YELLOW][2] = (11, 2)

        self.zoneEnterPosition[ZoneColor.BLUE] = dict()
        self.zoneEnterPosition[ZoneColor.BLUE][1] = (2, 1)
        self.zoneEnterPosition[ZoneColor.BLUE][2] = (0, 2)

        self.listZoneEnterPosition.append((6, 7))
        self.listZoneEnterPosition.append((4, 6))
        self.listZoneEnterPosition.append((5, 4))
        self.listZoneEnterPosition.append((7, 5))
        self.listZoneEnterPosition.append((2, 10))
        self.listZoneEnterPosition.append((0, 9))
        self.listZoneEnterPosition.append((9, 10))
        self.listZoneEnterPosition.append((11, 9))
        self.listZoneEnterPosition.append((9, 1))
        self.listZoneEnterPosition.append((11, 2))
        self.listZoneEnterPosition.append((2, 1))
        self.listZoneEnterPosition.append((0, 2))

    def SetEnterZonePositionMPC(self):
        self.zoneEnterPosition[ZoneColor.BLACK] = dict()
        self.zoneEnterPosition[ZoneColor.BLACK][1] = (6, 8)
        self.zoneEnterPosition[ZoneColor.BLACK][2] = (3, 6)
        self.zoneEnterPosition[ZoneColor.BLACK][3] = (5, 3)
        self.zoneEnterPosition[ZoneColor.BLACK][4] = (8, 5)

        self.zoneEnterPosition[ZoneColor.RED] = dict()
        self.zoneEnterPosition[ZoneColor.RED][1] = (3, 10)
        self.zoneEnterPosition[ZoneColor.RED][2] = (0, 8)

        self.zoneEnterPosition[ZoneColor.GREEN] = dict()
        self.zoneEnterPosition[ZoneColor.GREEN][1] = (8, 10)
        self.zoneEnterPosition[ZoneColor.GREEN][2] = (11, 8)

        self.zoneEnterPosition[ZoneColor.YELLOW] = dict()
        self.zoneEnterPosition[ZoneColor.YELLOW][1] = (8, 1)
        self.zoneEnterPosition[ZoneColor.YELLOW][2] = (11, 3)

        self.zoneEnterPosition[ZoneColor.BLUE] = dict()
        self.zoneEnterPosition[ZoneColor.BLUE][1] = (3, 1)
        self.zoneEnterPosition[ZoneColor.BLUE][2] = (0, 3)

        self.listZoneEnterPosition.append((6, 8))
        self.listZoneEnterPosition.append((3, 6))
        self.listZoneEnterPosition.append((5, 3))
        self.listZoneEnterPosition.append((8, 5))
        self.listZoneEnterPosition.append((3, 10))
        self.listZoneEnterPosition.append((0, 8))
        self.listZoneEnterPosition.append((8, 10))
        self.listZoneEnterPosition.append((11, 8))
        self.listZoneEnterPosition.append((8, 1))
        self.listZoneEnterPosition.append((11, 3))
        self.listZoneEnterPosition.append((3, 1))
        self.listZoneEnterPosition.append((0, 3))

    def SetPuckInZone(self):
        self.puckInZone[ZoneColor.BLACK] = dict()
        self.puckInZone[ZoneColor.BLACK][1] = 1
        self.puckInZone[ZoneColor.BLACK][2] = 1
        self.puckInZone[ZoneColor.BLACK][3] = 1
        self.puckInZone[ZoneColor.BLACK][4] = 1

        self.puckInZone[ZoneColor.RED] = dict()
        self.puckInZone[ZoneColor.RED][1] = 0
        self.puckInZone[ZoneColor.RED][2] = 0

        self.puckInZone[ZoneColor.GREEN] = dict()
        self.puckInZone[ZoneColor.GREEN][1] = 0
        self.puckInZone[ZoneColor.GREEN][2] = 0

        self.puckInZone[ZoneColor.YELLOW] = dict()
        self.puckInZone[ZoneColor.YELLOW][1] = 0
        self.puckInZone[ZoneColor.YELLOW][2] = 0

        self.puckInZone[ZoneColor.BLUE] = dict()
        self.puckInZone[ZoneColor.BLUE][1] = 0
        self.puckInZone[ZoneColor.BLUE][2] = 0

    def SearchNearestZone(self, zoneColor: ZoneColor, robotNumber):
        dist = np.inf
        aim = self.bots[robotNumber - 1].aimPosition
        weight_robot = 1.1

        if zoneColor == ZoneColor.BLACK:
            for zoneNumber in range(1, 5):
                if self.puckInZone[zoneColor][zoneNumber] == 1:
                    temp_enter = self.zoneEnterPosition[zoneColor][zoneNumber]
                    tempDist = self.TaxicabNorm(self.bots[robotNumber-1].robotPosition, temp_enter)

                    for i in range(len(self.bots)):
                        if i == robotNumber - 1:
                            continue
                        else:
                            if self.bots[i].aimPosition == temp_enter:
                                tempDist += weight_robot

                    print(tempDist)
                    if tempDist < dist:
                        dist = tempDist
                        aim = temp_enter

        else:
            for zoneNumber in range(1, 3):
                if self.puckInZone[zoneColor][zoneNumber] <= 24:
                    temp_enter = self.zoneEnterPosition[zoneColor][zoneNumber]
                    tempDist = self.TaxicabNorm(self.bots[robotNumber - 1].robotPosition, temp_enter)

                    for i in range(len(self.bots)):
                        if i == robotNumber - 1:
                            continue
                        else:
                            if self.bots[i].aimPosition == temp_enter:
                                tempDist += weight_robot

                    if tempDist < dist:
                        dist = tempDist
                        aim = temp_enter

        if zoneColor == ZoneColor.BLACK and np.isinf(dist):
            self.bots[robotNumber - 1].aimPosition = self.bots[robotNumber-1].finishPosition
            self.bots[robotNumber - 1].backToFinishPosition = True
            self.bots[robotNumber - 1].pathLength = 0

            '''
            x0 = list()
            for i in range(len(self.bots)):
                conv_pos = self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1])
                x0.append(conv_pos[0])
    
            for i in range(len(self.bots)):
                conv_pos = self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1])
                x0.append(conv_pos[1])
    
            if self.modelType == 2:
                pass
    
            else:
                for i in range(len(self.bots)):
                    x0.append(self.bots[i].globalOrientation)
    
            for i in range(len(self.bots)):
                x0.append(self.bots[i].pathLength)
    
            x0 = np.array(x0).reshape(-1, 1)
            self.mpc.x0 = x0
            self.mpc.set_initial_guess()
            '''
            return

        print(aim)
        self.bots[robotNumber-1].aimPosition = tuple(aim)
        self.bots[robotNumber - 1].pathLength = 0

        '''
        x0 = list()
        for i in range(len(self.bots)):
            conv_pos = self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1])
            x0.append(conv_pos[0])

        for i in range(len(self.bots)):
            conv_pos = self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1])
            x0.append(conv_pos[1])

        if self.modelType == 2:
            pass

        else:
            for i in range(len(self.bots)):
                x0.append(self.bots[i].globalOrientation)

        for i in range(len(self.bots)):
            x0.append(self.bots[i].pathLength)

        x0 = np.array(x0).reshape(-1, 1)
        self.mpc.x0 = x0
        self.mpc.set_initial_guess()
        '''

    def TaxicabNorm(self, position1, position2):
        return abs(position2[0] - position1[0]) + abs(position2[1] - position1[1])

    def Norm2(self, position1, position2):
        return sqrt((position2[0] - position1[0])**2 + (position2[1] - position1[1])**2)

    def Norm2Plus(self, position1, position2):
        return sqrt((position2[0] - position1[0])**2 + (position2[1] - position1[1])**2 + 0.00001)

    def MyNorm(self, position1, position2):
        return self.MyNormX(position1, position2) + self.MyNormY(position1, position2)

    def MyNormX(self, x, y):
        shift = 2
        dist_x = fabs(x[0] - y[0])
        group_x = self.SaturationFunctionGrid(x[0], 160, 5.5, -1, 1) * self.SaturationFunctionGrid(y[0], 200, 5.5, -1, 1)
        common_group_x = -self.SaturationFunctionGrid(group_x, 100, 0, -1, 0)
        correction_y0 = self.SaturationFunctionGrid(x[1], 160, 3.5, 0, 0.5) + \
                        self.SaturationFunctionGrid(x[1], 160, 4.5, 0, 0.5) - \
                        self.SaturationFunctionGrid(x[1], 160, 6.5, 0, 0.5) - \
                        self.SaturationFunctionGrid(x[1], 160, 7.5, 0, 0.5)

        correction_y1 = self.SaturationFunctionGrid(y[1], 160, 3.5, 0, 0.5) + \
                        self.SaturationFunctionGrid(y[1], 160, 4.5, 0, 0.5) - \
                        self.SaturationFunctionGrid(y[1], 160,  6.5, 0, 0.5) - \
                        self.SaturationFunctionGrid(y[1], 160, 7.5, 0, 0.5)

        group_y0 = self.SaturationFunctionGrid(x[1], 160, 3.5, 0, 1) - self.SaturationFunctionGrid(x[1], 160, 7.5, 0, 1)
        group_y1 = self.SaturationFunctionGrid(y[1], 160, 3.5, 0, 1) - self.SaturationFunctionGrid(y[1], 160, 7.5, 0, 1)

        dist_y0 = group_y0 * group_y1 * correction_y0 * shift * common_group_x
        dist_y1 = group_y0 * group_y1 * correction_y1 * shift * common_group_x
        dist_x += dist_y0 + dist_y1

        up_down = self.SaturationFunctionGrid(x[1], 160, 3.5, 0, 1) - \
                  self.SaturationFunctionGrid(x[1], 160, 3.5, 0, 2) + \
                  self.SaturationFunctionGrid(x[1], 160, 4.5, 0, 2) - \
                  self.SaturationFunctionGrid(x[1], 160, 5.5, 0, 2) + \
                  self.SaturationFunctionGrid(x[1], 160, 6.5, 0, 2) - \
                  self.SaturationFunctionGrid(x[1], 160, 7.5, 0, 1)

        up_down_correction = -self.SaturationFunctionGrid(up_down * (y[1] - x[1]), 160, -3.5, -1, 0) - \
                             self.SaturationFunctionGrid( up_down * (y[1] - x[1]), 160, -2.5, 0, 1) + \
                             self.SaturationFunctionGrid(up_down * (y[1] - x[1]), 160, -0.5, 0, 1) - \
                             self.SaturationFunctionGrid(up_down * (y[1] - x[1]), 160, 1.5, 0, 1) + \
                             self.SaturationFunctionGrid(up_down * (y[1] - x[1]), 160, 2.5, 0, 2)

        group_y1 = self.SaturationFunctionGrid(y[1], 160, 3.5, 0, 1) - self.SaturationFunctionGrid(y[1], 160, 7.5, 0, 1)
        dist_x += group_y1 * up_down_correction * common_group_x

        return dist_x

    def MyNormY(self, x, y):
        shift = 2
        dist_y = fabs(x[1] - y[1])
        group_y = self.SaturationFunctionGrid(x[1], 160, 5.5, -1, 1) * self.SaturationFunctionGrid(y[1], 200, 5.5, -1, 1)
        common_group_y = -self.SaturationFunctionGrid(group_y, 100, 0, -1, 0)
        correction_x0 = self.SaturationFunctionGrid(x[0], 160, 3.5, 0, 0.5) + \
                        self.SaturationFunctionGrid(x[0], 160, 4.5, 0, 0.5) - \
                        self.SaturationFunctionGrid(x[0], 160,  6.5, 0, 0.5) - \
                        self.SaturationFunctionGrid(x[0], 160, 7.5, 0, 0.5)

        correction_x1 = self.SaturationFunctionGrid(y[0], 160, 3.5, 0, 0.5) + \
                        self.SaturationFunctionGrid(y[0], 160, 4.5, 0, 0.5) - \
                        self.SaturationFunctionGrid(y[0], 160, 6.5, 0, 0.5) - \
                        self.SaturationFunctionGrid(y[0], 160, 7.5, 0, 0.5)

        group_x0 = self.SaturationFunctionGrid(x[0], 160, 3.5, 0, 1) - self.SaturationFunctionGrid(x[0], 160, 7.5, 0, 1)
        group_x1 = self.SaturationFunctionGrid(y[0], 160, 3.5, 0, 1) - self.SaturationFunctionGrid(y[0], 160, 7.5, 0, 1)

        dist_x0 = group_x0 * group_x1 * correction_x0 * shift * common_group_y
        dist_x1 = group_x0 * group_x1 * correction_x1 * shift * common_group_y
        dist_y += dist_x0 + dist_x1

        sign = -self.SaturationFunctionGrid(x[0], 160, 5.5, -1, 0) - self.SaturationFunctionGrid(x[0], 160, 5.5, 0, 1)

        left_right = self.SaturationFunctionGrid(x[0], 160, 3.5, 0, 1) - \
                     self.SaturationFunctionGrid(x[0], 160, 3.5, 0, 2) + \
                     self.SaturationFunctionGrid(x[0], 160, 4.5, 0, 2) - \
                     self.SaturationFunctionGrid(x[0], 160, 5.5, 0, 2) + \
                     self.SaturationFunctionGrid(x[0], 160, 6.5, 0, 2) - \
                     self.SaturationFunctionGrid(x[0], 160, 7.5, 0, 1)

        left_right_correction = -self.SaturationFunctionGrid(left_right * (y[0] - x[0]), 160, -3.5, -1, 0) - \
                                self.SaturationFunctionGrid(left_right * (y[0] - x[0]), 160, -2.5, 0, 1) + \
                                self.SaturationFunctionGrid(left_right * (y[0] - x[0]), 160, -0.5, 0, 1) - \
                                self.SaturationFunctionGrid(left_right * (y[0] - x[0]), 160, 1.5, 0, 1) + \
                                self.SaturationFunctionGrid(left_right * (y[0] - x[0]), 160, 2.5, 0, 2)

        dist_y += group_x1 * left_right_correction * common_group_y

        '''
        group_corner_x = self.SaturationFunctionGrid(y[0], 160, 2.5, 0, 1) - \
                          self.SaturationFunctionGrid(y[0], 160, 3.5, 0, 1) + \
                          self.SaturationFunctionGrid(y[0], 160, 7.5, 0, 1) - \
                          self.SaturationFunctionGrid(y[0], 160, 8.5, 0, 1)

        group_corner_y = self.SaturationFunctionGrid(y[1], 160, 2.5, 0, 1) - \
                          self.SaturationFunctionGrid(y[1], 160, 3.5, 0, 1) + \
                          self.SaturationFunctionGrid(y[1], 160, 7.5, 0, 1) - \
                          self.SaturationFunctionGrid(y[1], 160, 8.5, 0, 1)

        group_corner = group_corrner_x * group_corrner_y

        corner_correction = self.SaturationFunctionGrid(sign * (y[0] - x[0]), 160, 2.5, 0, 2) * common_group_y * group_corrner
        '''

        return dist_y

    def MyNorm2(self, x, y):
        norma = 0

        pole1_x = -self.SaturationFunctionGrid(x[0], 180, -1.29, -1, 0) * \
                  (-self.SaturationFunctionGrid(x[1], 180, 1.29, -1, 0)) + \
                  (self.SaturationFunctionGrid(x[0], 180, -1.29, 0, 1) -
                   self.SaturationFunctionGrid(x[0], 180, -0.5, 0, 1)) * \
                  (self.SaturationFunctionGrid(x[1], 180, -0.5, 0, 1) -
                   self.SaturationFunctionGrid(x[1], 180, 1.29, 0, 1))

        pole2_x = 2 * (-self.SaturationFunctionGrid(x[0], 180, 1.29, -1, 0) *
                       (self.SaturationFunctionGrid(x[1], 180, 1.29, 0, 1)) +
                       (self.SaturationFunctionGrid(x[0], 180, -0.5, 0, 1) -
                        self.SaturationFunctionGrid(x[0], 180, 1.29, 0, 1)) *
                       (self.SaturationFunctionGrid(x[1], 180, 0.5, 0, 1) -
                        self.SaturationFunctionGrid(x[1], 180, 1.29, 0, 1)))

        pole3_x = 3 * (self.SaturationFunctionGrid(x[0], 180, 1.29, 0, 1) *
                       (self.SaturationFunctionGrid(x[1], 180, -1.29, 0, 1)) +
                       (self.SaturationFunctionGrid(x[0], 180, 0.5, 0, 1) -
                        self.SaturationFunctionGrid(x[0], 180, 1.29, 0, 1)) *
                       (self.SaturationFunctionGrid(x[1], 180, -1.29, 0, 1) -
                        self.SaturationFunctionGrid(x[1], 180, 0.5, 0, 1)))

        pole4_x = 4 * (self.SaturationFunctionGrid(x[0], 180, -1.29, 0, 1) *
                       (-self.SaturationFunctionGrid(x[1], 180, -1.29, -1, 0)) +
                       (self.SaturationFunctionGrid(x[0], 180, -1.29, 0, 1) -
                        self.SaturationFunctionGrid(x[0], 180, 0.5, 0, 1)) *
                       (self.SaturationFunctionGrid(x[1], 180, -1.29, 0, 1) -
                        self.SaturationFunctionGrid(x[1], 180, -0.5, 0, 1)))

        pole1_y = -self.SaturationFunctionGrid(y[0], 180, -1.29, -1, 0) * (-self.SaturationFunctionGrid(y[1], 180, 1.29, -1, 0)) + (
                    self.SaturationFunctionGrid(y[0], 180, -1.29, 0, 1) - self.SaturationFunctionGrid(y[0], 180, -0.5, 0, 1)) * (
                              self.SaturationFunctionGrid(y[1], 180, -0.5, 0, 1) - self.SaturationFunctionGrid(y[1], 180, 1.29, 0, 1))
        pole2_y = 2 * (-self.SaturationFunctionGrid(y[0], 180, 1.29, -1, 0) * (self.SaturationFunctionGrid(y[1], 180, 1.29, 0, 1)) + (
                    self.SaturationFunctionGrid(y[0], 180, -0.5, 0, 1) - self.SaturationFunctionGrid(y[0], 180, 1.29, 0, 1)) * (
                                   self.SaturationFunctionGrid(y[1], 180, 0.5, 0, 1) - self.SaturationFunctionGrid(y[1], 180, 1.29, 0, 1)))
        pole3_y = 3 * (self.SaturationFunctionGrid(y[0], 180, 1.29, 0, 1) * (self.SaturationFunctionGrid(y[1], 180, -1.29, 0, 1)) + (
                    self.SaturationFunctionGrid(y[0], 180, 0.5, 0, 1) - self.SaturationFunctionGrid(y[0], 180, 1.29, 0, 1)) * (
                                   self.SaturationFunctionGrid(y[1], 180, -1.29, 0, 1) - self.SaturationFunctionGrid(y[1], 180, 0.5, 0, 1)))
        pole4_y = 4 * (self.SaturationFunctionGrid(y[0], 180, -1.29, 0, 1) * (-self.SaturationFunctionGrid(y[1], 180, -1.29, -1, 0)) + (
                    self.SaturationFunctionGrid(y[0], 180, -1.29, 0, 1) - self.SaturationFunctionGrid(y[0], 180, 0.5, 0, 1)) * (
                                   self.SaturationFunctionGrid(y[1], 180, -1.29, 0, 1) - self.SaturationFunctionGrid(y[1], 180, -0.5, 0, 1)))

        pole_x = pole1_x + pole2_x + pole3_x + pole4_x
        pole_y = pole1_y + pole2_y + pole3_y + pole4_y
        
        pole_3xy = 3 * pole_x * pole_y
        abs_pola = (0.5-0.5*(self.SaturationFunction(pole_x - pole_y, 180)))*(pole_x - pole_y)#fabs(pole_x - pole_y)
        
        rozklad_pol_w = self.SaturationFunctionGrid(abs_pola, 180, 1.5, 0, 1) - self.SaturationFunctionGrid(abs_pola, 180, 2.5, 0, 1)
        rozklad_pol_u = self.SaturationFunctionGrid(abs_pola, 180, 0.5, 0, 1)
        rozklad_pol_t = -self.SaturationFunctionGrid(abs_pola, 180, 1.5, -1, 0) + self.SaturationFunctionGrid(abs_pola, 180, 2.5, 0, 1)

        czy_podzial_pole_y_y = self.SaturationFunctionGrid(pole_y, 180, 0.5, 0, 1) - \
                               self.SaturationFunctionGrid(pole_y, 180, 1.5, 0, 1) + \
                               self.SaturationFunctionGrid(pole_y, 180, 2.5, 0, 1) - \
                               self.SaturationFunctionGrid(pole_y, 180, 3.5, 0, 1)

        czy_podzial_pole_y_x = self.SaturationFunctionGrid(pole_y, 180, 1.5, 0, 1) - \
                               self.SaturationFunctionGrid(pole_y, 180, 2.5, 0, 1) + \
                               self.SaturationFunctionGrid(pole_y, 180, 3.5, 0, 1)

        wsp_pole_y = -self.SaturationFunctionGrid(pole_y, 180, 2.5, -1, 1)

        podzial_pole_y_x = czy_podzial_pole_y_x * self.SaturationFunctionGrid(y[0], 250, wsp_pole_y * fabs(x[0] + 0.45),
                                                              -1 * wsp_pole_y, 1 * wsp_pole_y)
        podzial_pole_y_y = czy_podzial_pole_y_y * self.SaturationFunctionGrid(y[1], 250, -wsp_pole_y * fabs(x[1] - wsp_pole_y * 0.45),
                                                              -1 * wsp_pole_y, 1 * wsp_pole_y)

        podzial_pole_y = podzial_pole_y_x + podzial_pole_y_y

        naprzeciw_wartosc = 0.5 * pole_x * podzial_pole_y
        
        ktore_bramy = rozklad_pol_u * (pole_3xy + (rozklad_pol_w * naprzeciw_wartosc))

        brama_1_x = (y[0] / (fabs(y[0]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 35, -1, 0, fabs(y[0])) - \
                    (y[0] / (fabs(y[0]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 35, 3, 0, fabs(y[0])) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 3, 0, 0.75) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 6.75, 0, 1.95) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 8, 0,  2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 9, 0, 0.45) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 10, 0, 1.95) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 11.25, 0, 2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 15, 0, 2.4) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 20, 0, 2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 22.5, 0, 0.45) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 24, 0, 1.95)
        
        brama_1_y = (y[1] / (fabs(y[1]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 30, -1, 0, fabs(y[1])) - \
                    (y[1] / (fabs(y[1]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 30, 3, 0, fabs(y[1])) + \
                    self.SaturationFunctionGrid(ktore_bramy, 30, 3, 0, 1.2) - \
                    self.SaturationFunctionGrid(ktore_bramy, 30, 6.75, 0, 2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 30, 8, 0, 0.45) + \
                    self.SaturationFunctionGrid(ktore_bramy, 30, 9, 0, 1.95) - \
                    self.SaturationFunctionGrid(ktore_bramy, 30, 11.25, 0, 1.95) + \
                    self.SaturationFunctionGrid(ktore_bramy, 30, 15, 0, 1.95) - \
                    self.SaturationFunctionGrid(ktore_bramy, 30, 20, 0, 0.45) + \
                    self.SaturationFunctionGrid(ktore_bramy, 30, 22.5, 0, 0.45) - \
                    self.SaturationFunctionGrid(ktore_bramy, 30, 25.5, 0, 2.4)

        brama_2_x = (y[0] / (fabs(y[0]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 35, -1, 0, fabs(y[0])) - \
                    (y[0] / (fabs(y[0]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 35, 3, 0, fabs(y[0])) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 3, 0, 1.2) +  \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 6.75, 0, 1.95) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 8, 0, 1.95) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 10, 0, 2.4) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 11.25, 0, 2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 15, 0, 2.4) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 20, 0, 2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 22.5, 0, 0) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 24, 0, 2.4) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 25.5, 0, 0.45)

        brama_2_y = (y[1] / (fabs(y[1]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 35, -1, 0, fabs(y[1])) - \
                    (y[1] / (fabs(y[1]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 35, 3, 0, fabs(y[1])) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 3, 0, 1.2) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 6.75, 0, 2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 9, 0, 2.4) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 10, 0, 0.45) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 11.25, 0, 1.95) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 15, 0, 1.95) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 20, 0, 1.95) + \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 22.5, 0, 2.4) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 24, 0, 0.45) - \
                    self.SaturationFunctionGrid(ktore_bramy, 35, 25.5, 0, 1.95)


        '''
        brama_1_x = (y[0] / (fabs(y[0]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 180, -1, 0, fabs(y[0])) - \
                    (y[0] / (fabs(y[0]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 180, 3, 0, fabs(y[0])) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 3, 0, 0.75) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 6.75, 0, 1.95) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 8, 0,  2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 9, 0, 0.45) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 10, 0, 1.95) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 11.25, 0, 2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 15, 0, 2.4) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 20, 0, 2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 22.5, 0, 0.45) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 24, 0, 1.95)
        
        brama_1_y = (y[1] / (fabs(y[1]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 180, -1, 0, fabs(y[1])) - \
                    (y[1] / (fabs(y[1]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 180, 3, 0, fabs(y[1])) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 3, 0, 1.2) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 6.75, 0, 2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 8, 0, 0.45) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 9, 0, 1.95) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 11.25, 0, 1.95) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 15, 0, 1.95) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 20, 0, 0.45) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 22.5, 0, 0.45) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 25.5, 0, 2.4)

        brama_2_x = (y[0] / (fabs(y[0]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 180, -1, 0, fabs(y[0])) - \
                    (y[0] / (fabs(y[0]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 180, 3, 0, fabs(y[0])) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 3, 0, 1.2) +  \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 6.75, 0, 1.95) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 8, 0, 1.95) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 10, 0, 2.4) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 11.25, 0, 2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 15, 0, 2.4) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 20, 0, 2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 22.5, 0, 0) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 24, 0, 2.4) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 25.5, 0, 0.45)

        brama_2_y = (y[1] / (fabs(y[1]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 180, -1, 0, fabs(y[1])) - \
                    (y[1] / (fabs(y[1]) + 0.01)) * self.SaturationFunctionGrid(ktore_bramy, 180, 3, 0, fabs(y[1])) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 3, 0, 1.2) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 6.75, 0, 2.4) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 9, 0, 2.4) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 10, 0, 0.45) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 11.25, 0, 1.95) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 15, 0, 1.95) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 20, 0, 1.95) + \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 22.5, 0, 2.4) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 24, 0, 0.45) - \
                    self.SaturationFunctionGrid(ktore_bramy, 180, 25.5, 0, 1.95)

        '''

        norma1_x = fabs(brama_1_x - x[0])
        norma1_y = fabs(brama_1_y - x[1])

        norma2_x = fabs(brama_2_x - x[0])
        norma2_y = fabs(brama_2_y - x[1])

        norma1_2 = (norma1_x + norma1_y) - (norma2_x + norma2_y)
        
        czy_norma1 = -self.SaturationFunctionGrid(norma1_2, 180, 0, -1, 0)
        czy_norma2 = self.SaturationFunctionGrid(norma1_2, 180, 0, 0, 1)

        norma = czy_norma1 * (norma1_x + norma1_y) + czy_norma2 * (norma2_x + norma2_y)

        
        wariant1_norma1_x = fabs(brama_1_x - y[0])
        wariant1_norma1_y = fabs(brama_1_y - y[1])

        wariant1_norma2_x = fabs(brama_2_x - y[0])
        wariant1_norma2_y = fabs(brama_2_y - y[1])

        wariant1_norma = czy_norma1 * (wariant1_norma1_x + wariant1_norma1_y) + czy_norma2 * (
                    wariant1_norma2_x + wariant1_norma2_y)

        brama_koncowa = pole_y * podzial_pole_y
        brama_pole_y_x = self.SaturationFunctionGrid(brama_koncowa, 80, -3.5, -1.2, 0) + \
                         self.SaturationFunctionGrid(brama_koncowa, 80, -3.5, 0, 0.75) - \
                         self.SaturationFunctionGrid(brama_koncowa, 80, -2.5, 0, 1.95) + \
                         self.SaturationFunctionGrid(brama_koncowa, 80, 0, 0, 0.45) + \
                         self.SaturationFunctionGrid(brama_koncowa, 80, 1.5, 0, 1.95)

        brama_pole_y_y = self.SaturationFunctionGrid(brama_koncowa, 80, -3.5, -0.75, 0) - \
                         self.SaturationFunctionGrid(brama_koncowa, 80, -3.5, 0, 1.2) + \
                         self.SaturationFunctionGrid(brama_koncowa, 80, -2.5, 0, 2.4) - \
                         self.SaturationFunctionGrid(brama_koncowa, 80, -1.5, 0, 2.4) + \
                         self.SaturationFunctionGrid(brama_koncowa, 80, 0, 0, 2.4) - \
                         self.SaturationFunctionGrid(brama_koncowa, 80, 1.5, 0, 0.45) + \
                         self.SaturationFunctionGrid(brama_koncowa, 80, 2.5, 0, 0.45) - \
                         self.SaturationFunctionGrid(brama_koncowa, 80, 3.5, 0, 2.4)

        wariant2_norma1_x = fabs(brama_1_x - brama_pole_y_x) + fabs(brama_pole_y_x - y[0])
        wariant2_norma1_y = fabs(brama_1_y - brama_pole_y_y) + fabs(brama_pole_y_y - y[1])

        wariant2_norma2_x = fabs(brama_2_x - brama_pole_y_x) + fabs(brama_pole_y_x - y[0])
        wariant2_norma2_y = fabs(brama_2_y - brama_pole_y_y) + fabs(brama_pole_y_y - y[1])

        wariant2_norma = czy_norma1 * (wariant2_norma1_x + wariant2_norma1_y) + czy_norma2 * (
                    wariant2_norma2_x + wariant2_norma2_y)

        norma = norma + rozklad_pol_u * rozklad_pol_t * (wariant1_norma) + rozklad_pol_u * rozklad_pol_w * (
            wariant2_norma)
        
        #blokada_srodka = -self.SaturationFunctionGrid(pole_y, 250, 0.3, -10, 0)
        #norma += blokada_srodka
        
        return norma

    def SaturationFunction(self, x, scale=1):
        return 1-((scale*x)/(sqrt(1+(scale*x)**2)))

    def SaturationFunctionGrid(self, x, scale, shift, y_min=0.0, y_max=1.0):
        return y_max - (y_max - y_min) / (1 + exp(scale * (x - shift) / (y_max - y_min)))

    def GetNeighbors(self, position):
        x, y = position
        result = [(x, y+1), (x+1, y), (x, y-1), (x-1, y)] #N E S W

        if x == 0:
            result.remove((x-1, y))
        elif x == 11:
            result.remove((x+1, y))

        if y == 0:
            result.remove((x, y - 1))
        elif y == 11:
            result.remove((x, y + 1))

        toRemove = []
        for i in result:
            if self.robotMap[i[0]][i[1]] != 0:
                toRemove.append(i)

        for i in toRemove:
            if i in result:
                result.remove(i)

        #For MPC not need
        '''
        if x == 5 and y == 4:
            result = [(x, y-1)]

        elif x == 4 and y == 6:
            result = [(x-1, y)]

        elif x == 6 and y == 7:
            result = [(x, y+1)]

        elif x == 7 and y == 5:
            result = [(x+1, y)]

        elif x == 4 and y == 4:
            result.remove((x+1, y))

        elif x == 6 and y == 4:
            result.remove((x-1, y))

        elif x == 7 and y == 4:
            result.remove((x, y+1))

        elif x == 7 and y == 6:
            result.remove((x, y-1))

        elif x == 7 and y == 7:
            result.remove((x-1, y))

        elif x == 5 and y == 7:
            result.remove((x+1, y))

        elif x == 4 and y == 7:
            result.remove((x, y-1))

        elif x == 4 and y == 5:
            result.remove((x, y+1))
        '''

        return result

    def GetNeighborsDistatnce(self, position):
        x, y = position
        conv_pos = self.ConvertOnRealCoordinates(x, y)
        result = [(x, y+1), (x+1, y), (x, y-1), (x-1, y)] #N E S W

        if x == 0:
            result.remove((x-1, y))
        elif x == 11:
            result.remove((x+1, y))

        if y == 0:
            result.remove((x, y - 1))
        elif y == 11:
            result.remove((x, y + 1))


        for i in range(len(result)):
            if self.robotMap[result[i][0]][result[i][1]] != 0:
                result[i] = None
            else:
                conv_pos_neighbors = self.ConvertOnRealCoordinates(result[i][0], result[i][1])
                result[i] = self.TaxicabNorm(conv_pos, conv_pos_neighbors)


        #For MPC not need, if you need change on distance !!!!
        '''
        if x == 5 and y == 4:
            result = [None, None, 0.675, None]

        elif x == 4 and y == 6:
            result = [(x-1, y)]

        elif x == 6 and y == 7:
            result = [(x, y+1)]

        elif x == 7 and y == 5:
            result = [(x+1, y)]

        elif x == 4 and y == 4:
            result.remove((x+1, y))

        elif x == 6 and y == 4:
            result.remove((x-1, y))

        elif x == 7 and y == 4:
            result.remove((x, y+1))

        elif x == 7 and y == 6:
            result.remove((x, y-1))

        elif x == 7 and y == 7:
            result.remove((x-1, y))

        elif x == 5 and y == 7:
            result.remove((x+1, y))

        elif x == 4 and y == 7:
            result.remove((x, y-1))

        elif x == 4 and y == 5:
            result.remove((x, y+1))
        '''

        return result

    def GetNeighborsDistatnceWithoutRobots(self, position):
        x, y = position
        conv_pos = self.ConvertOnRealCoordinates(x, y)
        result = [(x, y+1), (x+1, y), (x, y-1), (x-1, y)] #N E S W

        if x == 0:
            result[3] = None
        elif x == 11:
            result[1] = None

        if y == 0:
            result[2] = None
        elif y == 11:
            result[0] = None


        for i in range(len(result)):
            if result[i] != None:
                if self.robotMap[result[i][0]][result[i][1]] == -1:
                    result[i] = None
                else:
                    conv_pos_neighbors = self.ConvertOnRealCoordinates(result[i][0], result[i][1])
                    result[i] = self.TaxicabNorm(conv_pos, conv_pos_neighbors)


        #For MPC not need, if you need change on distance !!!!
        '''
        if x == 5 and y == 4:
            result = [None, None, 0.675, None]

        elif x == 4 and y == 6:
            result = [(x-1, y)]

        elif x == 6 and y == 7:
            result = [(x, y+1)]

        elif x == 7 and y == 5:
            result = [(x+1, y)]

        elif x == 4 and y == 4:
            result.remove((x+1, y))

        elif x == 6 and y == 4:
            result.remove((x-1, y))

        elif x == 7 and y == 4:
            result.remove((x, y+1))

        elif x == 7 and y == 6:
            result.remove((x, y-1))

        elif x == 7 and y == 7:
            result.remove((x-1, y))

        elif x == 5 and y == 7:
            result.remove((x+1, y))

        elif x == 4 and y == 7:
            result.remove((x, y-1))

        elif x == 4 and y == 5:
            result.remove((x, y+1))
        '''

        return result

    def GetNeighborsFlagsForm(self, position):
        x, y = position
        result = [(x, y+1), (x+1, y), (x, y-1), (x-1, y)] #N W S E

        if x == 0:
            result.remove((x-1, y))
        elif x == 11:
            result.remove((x+1, y))

        if y == 0:
            result.remove((x, y - 1))
        elif y == 11:
            result.remove((x, y + 1))

        toRemove = []
        for i in result:
            if self.robotMap[i[0]][i[1]] != 0:
                toRemove.append(i)

        for i in toRemove:
            result.remove(i)

        #For MPC not need
        '''
        if x == 5 and y == 4:
            result = [(x, y-1)]

        elif x == 4 and y == 6:
            result = [(x-1, y)]

        elif x == 6 and y == 7:
            result = [(x, y+1)]

        elif x == 7 and y == 5:
            result = [(x+1, y)]

        elif x == 4 and y == 4:
            result.remove((x+1, y))

        elif x == 6 and y == 4:
            result.remove((x-1, y))

        elif x == 7 and y == 4:
            result.remove((x, y+1))

        elif x == 7 and y == 6:
            result.remove((x, y-1))

        elif x == 7 and y == 7:
            result.remove((x-1, y))

        elif x == 5 and y == 7:
            result.remove((x+1, y))

        elif x == 4 and y == 7:
            result.remove((x, y-1))

        elif x == 4 and y == 5:
            result.remove((x, y+1))
        '''
        north = 0
        east = 0
        south = 0
        west = 0

        for i in result:
            if i[1] > y:
                north = 1
                continue

            if i[0] > x:
                west = 1
                continue

            if i[1] < y:
                south = 1
                continue

            if i[0] < x:
                east = 1
                continue

        return [north, west, south, east]

    def CalculationOnePathStep(self):

        tempReadyBots = 0

        for i in range(len(self.bots)):
            if self.bots[i].canMakeNewStep or self.botsFinished[i]:
                tempReadyBots += 1

        if not (tempReadyBots == len(self.bots)):
            return

        self.planning_cycle_counter += 1

        x0 = list()
        for i in range(len(self.bots)):
            conv_pos = self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1])
            x0.append(conv_pos[0])

        for i in range(len(self.bots)):
            conv_pos = self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1])
            x0.append(conv_pos[1])

        if self.modelType == 2:
            pass

        else:
            for i in range(len(self.bots)):
                x0.append(self.bots[i].globalOrientation)

        for i in range(len(self.bots)):
            x0.append(self.bots[i].pathLength)

        x0 = np.array(x0).reshape(-1, 1)

        self.mpc.x0 = x0
        self.u0 = self.mpc.make_step(x0)
        '''
        mpl.rcParams['font.size'] = 18
        mpl.rcParams['lines.linewidth'] = 3
        mpl.rcParams['axes.grid'] = True

        mpc_graphics = do_mpc.graphics.Graphics(self.mpc.data)

        fig, ax = plt.subplots(2, sharex=True, figsize=(16, 9))
        fig.align_ylabels()

        for g in [mpc_graphics]:
            g.add_line(var_type='_x', var_name='pos_x', axis=ax[0])
            g.add_line(var_type='_x', var_name='pos_y', axis=ax[0])
            g.add_line(var_type='_x', var_name='theta', axis=ax[0])

            g.add_line(var_type='_u', var_name='v', axis=ax[1])
            g.add_line(var_type='_u', var_name='omega', axis=ax[1])

        ax[0].set_ylabel('location')
        ax[1].set_ylabel('velocity')
        ax[1].set_xlabel('time [s]')

        mpc_graphics.plot_predictions()
        mpc_graphics.reset_axes()
        
        fig.show()
        input()
        '''

        '''
        pre_x = self.mpc.data.prediction(('_x', 'pos_x'))
        pre_y = self.mpc.data.prediction(('_x', 'pos_y'))
        distance = self.mpc.data.prediction(('_x', 'distance'))

        pre_pos = []
        pre_distance = []
        for j in range(len(self.bots)):
            pre_pos.append([])
            pre_distance.append([])
            for i in range(len(pre_x[j])):
                pre_pos[j].append((pre_x[j][i][0], pre_y[j][i][0]))
                pre_distance[j].append(distance[j][i][0])

        w_lterm_dist = 55
        w_lterm_path = 10
        w_lterm_avoid = 15
        w_mterm_dist = 80
        w_mterm = 6
        w_lterm = 1

        mterm = 0
        for i in range(len(self.bots)):
            mterm += w_mterm_dist*abs(pre_pos[i][len(pre_pos[i])-1][0] - self.bots[i].aimPosition[0]) + w_mterm_dist*abs(
                pre_pos[i][len(pre_pos[i])-1][1] - self.bots[i].aimPosition[1])

        #mterm /= 2*len(self.bots)*w_mterm_dist

        lterm = []
        for k in range(len(pre_pos[0])-1):
            lterm.append(0)
            for i in range(len(self.bots)):
                lterm[k] += w_lterm_dist*abs(pre_pos[i][k][0] - self.bots[i].aimPosition[0]) \
                          + w_lterm_dist*abs(pre_pos[i][k][1] - self.bots[i].aimPosition[1])

                #lterm += self.Norm2([model.x['pos_x', i], model.x['pos_y', i]], [model.tvp['aim_x', i],  model.tvp['aim_y', i]])
                lterm[k] += w_lterm_path * pre_distance[i][k]

            for i in range(len(self.bots)):
                for j in range(len(self.bots)):
                    if i == j:
                        continue
                    else:
                        pass
                        lterm[k] += w_lterm_avoid*self.SaturationFunction(abs(pre_pos[i][k][0] - pre_pos[j][k][0]), 2)\
                                 + w_lterm_avoid*self.SaturationFunction(abs(pre_pos[i][k][1] - pre_pos[j][k][1]), 2)
                        #lterm += -15*sqrt((model.x['pos_x', i] - model.x['pos_x', j])**2 + (model.x['pos_y', i] - model.x['pos_y', j])**2)


            #lterm[k] /= (2*len(self.bots)*w_lterm_dist)+(2*(len(self.bots)-1)*len(self.bots)*w_lterm_avoid)

            lterm[k] /= len(pre_pos[i])-1
            lterm[k] *= 0.5

            #lterm[k] *= w_lterm / (w_lterm + w_mterm)
        #mterm *= w_mterm / (w_lterm + w_mterm)

        print('\033[91mCost function value: {}\033[0m'.format((sum(lterm)+mterm)))

        print('Controll: {}'.format(self.u0))
        '''

        for i in range(len(self.bots)):
            self.bots[i].newPathStep = True

        self.Logger()

    def MakeOneStep(self, robotNumber):
        robotState = self.bots[robotNumber - 1].robotState
        if robotState == RobotState.READY:

            self.bots[robotNumber - 1].robotState = RobotState.FORWARD


            conv_pos = self.ConvertOnRealCoordinates(self.bots[robotNumber - 1].robotPosition[0],
                                                     self.bots[robotNumber - 1].robotPosition[1])

            dtime = 0.5
            if self.modelType == 2:
                est_theta = self.u0[len(self.bots) + (robotNumber - 1)]
            else:
                est_theta = self.bots[robotNumber - 1].globalOrientation + dtime * self.u0[len(self.bots)+(robotNumber - 1)]


            x = conv_pos[0] + dtime * self.u0[(robotNumber - 1)] * cos(est_theta)
            y = conv_pos[1] + dtime * self.u0[(robotNumber - 1)] * sin(est_theta)

            neighbors = self.GetNeighbors(self.bots[robotNumber - 1].robotPosition)
            neighbors.append(self.bots[robotNumber-1].robotPosition)


            pre_x = self.mpc.data.prediction(('_x', 'pos_x'))
            pre_y = self.mpc.data.prediction(('_x', 'pos_y'))

            pre_pos = []
            for i in range(len(pre_x[robotNumber - 1])):
                pre_pos.append((pre_x[robotNumber - 1][i][0], pre_y[robotNumber - 1][i][0]))
                print('Robot: {}, prediction {}: {}, fg: {:.5}'.format(robotNumber, i, pre_pos[i], self.functionGrid2(pre_pos[i][0], pre_pos[i][1])))

            current_pos = self.ConvertOnRealCoordinates(self.bots[robotNumber - 1].robotPosition[0],
                                                        self.bots[robotNumber - 1].robotPosition[1])
            current_x = current_pos[0]
            current_y = current_pos[1]

            neighbors = self.GetNeighborsFlagsForm(self.bots[robotNumber - 1].robotPosition)
            neighbors_dist = self.GetNeighborsDistatnceWithoutRobots(self.bots[robotNumber - 1].robotPosition)

            pre_pos[1] = (x[0], y[0])
            self.bots[robotNumber - 1].nextStep = self.bots[robotNumber - 1].robotPosition

            print('Current pos: ', (current_x, current_y))
            pre_length = len(pre_pos) - 1

            for i in range(1, pre_length):
                temp_x = pre_pos[i][0]
                temp_y = pre_pos[i][1]

                print('Temp pos[{}]: {}'.format(i, (temp_x, temp_y)))
                print('Difference pos[{}]: {}'.format(i, (abs(current_x - temp_x), abs(current_y - temp_y))))

                if abs(current_x - temp_x) >= abs(current_y - temp_y):
                    if temp_x > current_x:

                        threshold = np.inf
                        if neighbors_dist[2] != None:
                            threshold = neighbors_dist[2] / 2

                        print('Top threshold: ', current_x + threshold)

                        if temp_x > current_x + threshold:
                            if neighbors[2] == 1:
                                self.bots[robotNumber - 1].nextStep = (
                                self.bots[robotNumber - 1].nextStep[0], self.bots[robotNumber - 1].nextStep[1] - 1)
                                break
                            else:
                                break

                    else:

                        threshold = np.inf
                        if neighbors_dist[0] != None:
                            threshold = neighbors_dist[0] / 2

                        print('Bottom threshold: ', current_x - threshold)

                        if temp_x < current_x - threshold:
                            if neighbors[0] == 1:
                                self.bots[robotNumber - 1].nextStep = (
                                self.bots[robotNumber - 1].nextStep[0], self.bots[robotNumber - 1].nextStep[1] + 1)
                                break
                            else:
                                break
                else:
                    if temp_y > current_y:

                        threshold = np.inf
                        if neighbors_dist[3] != None:
                            threshold = neighbors_dist[3] / 2

                        print('Left threshold: ', current_y + threshold)

                        if temp_y > current_y + threshold:
                            if neighbors[3] == 1:
                                self.bots[robotNumber - 1].nextStep = (
                                self.bots[robotNumber - 1].nextStep[0] - 1, self.bots[robotNumber - 1].nextStep[1])
                                break
                            else:
                                break
                    else:

                        threshold = np.inf
                        if neighbors_dist[1] != None:
                            threshold = neighbors_dist[1] / 2

                        print('Right threshold: ', current_y - threshold)

                        if temp_y < current_y - threshold:
                            if neighbors[1] == 1:
                                self.bots[robotNumber - 1].nextStep = (
                                self.bots[robotNumber - 1].nextStep[0] + 1, self.bots[robotNumber - 1].nextStep[1])
                                break
                            else:
                                break


            if self.bots[robotNumber - 1].nextStep == self.bots[robotNumber - 1].robotPosition:
                print('Deadlock counter: {}'.format(self.bots[robotNumber - 1].stopCounter))

                for i in range(pre_length, len(pre_pos)):
                    temp_x = pre_pos[i][0]
                    temp_y = pre_pos[i][1]

                    if abs(current_x - temp_x) >= abs(current_y - temp_y):
                        if temp_x > current_x:

                            threshold = np.inf
                            if neighbors_dist[2] != None:
                                threshold = neighbors_dist[2] / 2

                            if temp_x > current_x + threshold:
                                if neighbors[2] == 1:
                                    self.bots[robotNumber - 1].nextStep = (
                                        self.bots[robotNumber - 1].nextStep[0],
                                        self.bots[robotNumber - 1].nextStep[1] - 1)
                                    break
                                else:
                                    break

                        else:

                            threshold = np.inf
                            if neighbors_dist[0] != None:
                                threshold = neighbors_dist[0] / 2

                            if temp_x < current_x - threshold:
                                if neighbors[0] == 1:
                                    self.bots[robotNumber - 1].nextStep = (
                                        self.bots[robotNumber - 1].nextStep[0],
                                        self.bots[robotNumber - 1].nextStep[1] + 1)
                                    break
                                else:
                                    break
                    else:
                        if temp_y > current_y:

                            threshold = np.inf
                            if neighbors_dist[3] != None:
                                threshold = neighbors_dist[3] / 2

                            if temp_y > current_y + threshold:
                                if neighbors[3] == 1:
                                    self.bots[robotNumber - 1].nextStep = (
                                        self.bots[robotNumber - 1].nextStep[0] - 1,
                                        self.bots[robotNumber - 1].nextStep[1])
                                    break
                                else:
                                    break
                        else:

                            threshold = np.inf
                            if neighbors_dist[1] != None:
                                threshold = neighbors_dist[1] / 2

                            if temp_y < current_y - threshold:
                                if neighbors[1] == 1:
                                    self.bots[robotNumber - 1].nextStep = (
                                        self.bots[robotNumber - 1].nextStep[0] + 1,
                                        self.bots[robotNumber - 1].nextStep[1])
                                    break
                                else:
                                    break

            if self.bots[robotNumber - 1].nextStep == self.bots[robotNumber - 1].robotPosition:
                self.bots[robotNumber - 1].stopCounter += 1

            else:
                self.bots[robotNumber - 1].stopCounter = 0

            if self.bots[robotNumber - 1].stopCounter >= random.randint(3, 6):
                neighbors = self.GetNeighbors(self.bots[robotNumber - 1].robotPosition)
                self.bots[robotNumber - 1].nextStep = neighbors[random.randint(0, len(neighbors)-1)]

            '''
            neighbors_weight = [0]*len(neighbors)
            dist = math.inf
            index = 0
            min_index = 0

            for i in neighbors:
                conv_i = self.ConvertOnRealCoordinates(i[0], i[1])

                if self.TaxicabNorm(conv_i, (x, y)) < dist:
                    dist = self.TaxicabNorm(conv_i, (x, y))
                    min_index = index

                index += 1

            neighbors_weight[min_index] += 1

            pre_step = 4
            for j in range(pre_step):
                index = 0
                min_index = 0
                dist = math.inf

                for i in neighbors:
                    conv_i = self.ConvertOnRealCoordinates(i[0], i[1])
                    temp_norm = self.TaxicabNorm(conv_i, (pre_pos[6 + (j*2)][0], pre_pos[2 + (j*2)][1]))
                    if temp_norm < dist:
                        dist = temp_norm
                        min_index = index

                    index += 1

                neighbors_weight[min_index] += 1#(1-(j/10))


            index = 0
            min_index = 0
            dist = math.inf

            for i in neighbors:
                conv_i = self.ConvertOnRealCoordinates(i[0], i[1])
                temp_norm = self.TaxicabNorm(conv_i, (pre_pos[19][0], pre_pos[19][1]))
                if temp_norm < dist:
                    dist = temp_norm
                    min_index = index

                index += 1

            neighbors_weight[min_index] += 2

            [print('neighbors_weight({}): {}'.format(neighbors[i], neighbors_weight[i])) for i in range(len(neighbors))]
            self.bots[robotNumber - 1].nextStep = neighbors[neighbors_weight.index(max(neighbors_weight))]
            '''
            '''
            current_pos = self.ConvertOnRealCoordinates(self.bots[robotNumber - 1].robotPosition[0], self.bots[robotNumber - 1].robotPosition[1])
            current_x = current_pos[0]
            current_y = current_pos[1]

            gradients_x = [(x-current_x)[0], (pre_pos[2][0]-x)[0]]
            gradients_y = [(y-current_y)[0], (pre_pos[2][1]-y)[0]]


            all_gradients_x = [(x-current_x)[0], (pre_pos[2][0]-x)[0]]
            all_gradients_y = [(y-current_y)[0], (pre_pos[2][1]-y)[0]]

            for i in range(len(pre_pos)-3):
                all_gradients_x.append(pre_pos[2+i+1][0] - pre_pos[2+i][0])
                all_gradients_y.append(pre_pos[2+i+1][1] - pre_pos[2+i][1])

            print('\033[93mMean gradient x: {}\033[0m'.format(sum(all_gradients_x) / len(all_gradients_x)))
            print('\033[93mMean gradient y: {}\033[0m'.format(sum(all_gradients_y) / len(all_gradients_y)))

            pre_step = 12
            for i in range(pre_step):
                gradients_x.append(pre_pos[2+i+1][0] - pre_pos[2+i][0])
                gradients_y.append(pre_pos[2+i+1][1] - pre_pos[2+i][1])

            sum_gradients_x = sum(gradients_x)
            sum_gradients_y = sum(gradients_y)

            threshold_x = 0.09
            threshold_y = 0.09

            neighbors = self.GetNeighborsFlagsForm(self.bots[robotNumber - 1].robotPosition)
            self.bots[robotNumber - 1].nextStep = self.bots[robotNumber - 1].robotPosition

            print('abs(sum_x): {} >= abs(sum_y): {}'.format(abs(sum_gradients_x), abs(sum_gradients_y)))
            if abs(sum_gradients_x) >= abs(sum_gradients_y):
                if sum_gradients_x < 0 and sum_gradients_x < -threshold_x and neighbors[0]:
                    self.bots[robotNumber-1].nextStep = (self.bots[robotNumber-1].nextStep[0], self.bots[robotNumber-1].nextStep[1]+1)

                elif sum_gradients_x >= 0 and sum_gradients_x > threshold_x and neighbors[2]:
                    self.bots[robotNumber-1].nextStep = (self.bots[robotNumber-1].nextStep[0], self.bots[robotNumber-1].nextStep[1]-1)

                elif sum_gradients_y < 0 and sum_gradients_y < -threshold_y and neighbors[1]:
                    self.bots[robotNumber-1].nextStep = (self.bots[robotNumber-1].nextStep[0]+1, self.bots[robotNumber-1].nextStep[1])

                elif sum_gradients_y >= 0 and sum_gradients_y >= threshold_y and neighbors[3]:
                    self.bots[robotNumber-1].nextStep = (self.bots[robotNumber-1].nextStep[0]-1, self.bots[robotNumber-1].nextStep[1])

            else:
                if sum_gradients_y < 0 and sum_gradients_y < -threshold_y and neighbors[1]:
                    self.bots[robotNumber - 1].nextStep = (
                        self.bots[robotNumber - 1].nextStep[0] + 1, self.bots[robotNumber - 1].nextStep[1])

                elif sum_gradients_y >= 0 and sum_gradients_y >= threshold_y and neighbors[3]:
                    self.bots[robotNumber - 1].nextStep = (
                        self.bots[robotNumber - 1].nextStep[0] - 1, self.bots[robotNumber - 1].nextStep[1])

                elif sum_gradients_x < 0 and sum_gradients_x < -threshold_x and neighbors[0]:
                    self.bots[robotNumber - 1].nextStep = (
                    self.bots[robotNumber - 1].nextStep[0], self.bots[robotNumber - 1].nextStep[1] + 1)

                elif sum_gradients_x >= 0 and sum_gradients_x > threshold_x and neighbors[2]:
                    self.bots[robotNumber - 1].nextStep = (
                    self.bots[robotNumber - 1].nextStep[0], self.bots[robotNumber - 1].nextStep[1] - 1)

            print('Gradiens x: {}'.format(gradients_x))
            print('Gradiens y: {}'.format(gradients_y))

            print('Sum gradiet x: {}'.format(sum_gradients_x))
            print('Sum gradiet y: {}'.format(sum_gradients_y))

            print('Mean gradient x: {}'.format(sum(gradients_x) / len(gradients_x)))
            print('Mean gradient y: {}'.format(sum(gradients_y) / len(gradients_y)))

            print('NextStep: {}'.format(self.bots[robotNumber - 1].nextStep))
            '''
            '''
            for i in neighbors:
                conv_i = self.ConvertOnRealCoordinates(i[0], i[1])
                #print('Coor: {}, dist: {}'.format(i, self.TaxicabNorm(conv_i, (x, y))))
                if self.TaxicabNorm(conv_i, (x, y)) < dist:
                    dist = self.TaxicabNorm(conv_i, (x, y))
                    self.bots[robotNumber - 1].nextStep = i
                    min_index = index

                index += 1


            pre_step = 3
            for j in range(pre_step):
                #print('\nmin_index first: ', min_index)
                if min_index == len(neighbors)-1:
                    index = 0
                    min_index = 0
                    dist = math.inf

                    for i in neighbors:
                        conv_i = self.ConvertOnRealCoordinates(i[0], i[1])
                        temp_norm = self.TaxicabNorm(conv_i, (pre_pos[1+j][0], pre_pos[1+j][1]))
                        #print('Coor for first prediction: {}, dist: {}'.format(i, temp_norm))
                        if temp_norm < dist:
                            dist = temp_norm
                            self.bots[robotNumber - 1].nextStep = i
                            min_index = index
                            print('iter {}: min_index {}'.format(j, min_index))

                        index += 1

                else:
                    break

            '''
            '''
            neighbors = self.GetNeighborsFlagsForm(self.bots[robotNumber - 1].robotPosition) #N W S E
            radius = 0.05

            if y >= conv_pos[1]+radius and x >= conv_pos[0]-radius and x <= conv_pos[0]+radius:
                if neighbors[3]:
                    self.bots[robotNumber - 1].robotPosition = (self.bots[robotNumber - 1].robotPosition[0]-1,
                                                     self.bots[robotNumber - 1].robotPosition[1])

            elif x <= conv_pos[0]-radius and y >= conv_pos[1] - radius and y <= conv_pos[1] + radius:
                if neighbors[0]:
                    self.bots[robotNumber - 1].robotPosition = (self.bots[robotNumber - 1].robotPosition[0],
                                                                self.bots[robotNumber - 1].robotPosition[1]+1)

            elif y <= conv_pos[1]-radius and x >= conv_pos[0]-radius and x <= conv_pos[0]+radius:
                if neighbors[1]:
                    self.bots[robotNumber - 1].robotPosition = (self.bots[robotNumber - 1].robotPosition[0]+1,
                                                     self.bots[robotNumber - 1].robotPosition[1])

            elif x >= conv_pos[0]+radius and y >= conv_pos[1] - radius and y <= conv_pos[1] + radius:
                if neighbors[2]:
                    self.bots[robotNumber - 1].robotPosition = (self.bots[robotNumber - 1].robotPosition[0],
                                                                self.bots[robotNumber - 1].robotPosition[1]-1)
            
            '''


            print('Controll: v: {}, omega: {}'.format(self.u0[(robotNumber - 1)], self.u0[len(self.bots)+(robotNumber - 1)]))

            print('x: ', x)
            print('y: ', y)
            print('Neighbors: ', neighbors)
            print('Robot{} pos: {}'.format(robotNumber, self.bots[robotNumber - 1].nextStep))
            print('Path distance: {}'.format(self.bots[robotNumber-1].pathLength))

            conv_pos = self.ConvertOnRealCoordinates(self.bots[robotNumber - 1].nextStep[0],
                                                    self.bots[robotNumber - 1].nextStep[1])

            self.robotMap[self.bots[robotNumber - 1].nextStep[0]][self.bots[robotNumber - 1].nextStep[1]] = robotNumber
            #self.bots[robotNumber - 1].sendCommand(robotNumber, str(RobotCommand.MOVE_TO_POSITION), [conv_pos[0], conv_pos[1]])

            self.bots[robotNumber - 1].sendCommand(robotNumber, str(RobotCommand.MOVE_WITH_ORIENTATION), [conv_pos[0], conv_pos[1], est_theta[0]])

    def ConvertOnRealCoordinates(self, i, j):
        x_coord = [2.85, 2.4, 1.8, 1.2, 0.75, 0.3, -0.3, -0.75, -1.2, -1.8, -2.4, -2.85]
        y_coord = [2.7, 2.25, 1.8, 1.2, 0.75, 0.3, -0.3, -0.75, -1.2, -1.8, -2.25, -2.7]

        if i == 0 and j == 2:
            return [1.875, 2.7]
        elif i == 2 and j == 1:
            return [2.4, 1.875]

        elif i == 0 and j == 9:
            return [-1.875, 2.7]
        elif i == 2 and j == 10:
            return [-2.4, 1.875]

        elif i == 9 and j == 1:
            return [2.4, -1.875]
        elif i == 11 and j == 2:
            return [1.875, -2.7]

        elif i == 11 and j == 9:
            return [-1.875, -2.7]
        elif i == 9 and j == 10:
            return [-2.4, -1.875]

        elif i == 5 and j == 4:
            return [0.525, 0.3]
        elif i == 4 and j == 6:
            return [-0.3, 0.525]
        elif i == 7 and j == 5:
            return [0.3, -0.525]
        elif i == 6 and j == 7:
            return [-0.525, -0.3]

        return [x_coord[j], y_coord[i]]

    def ConvertOnMapCoordinates(self, x):
        y_coord = {2.85: 0, 2.4: 1, 1.8: 2, 1.2: 3, 0.75: 4, 0.3: 5, -0.3: 6, -0.75: 7, -1.2: 8, -1.8: 9, -2.4: 10, -2.85: 11}
        x_coord = {2.7: 0, 2.25: 1, 1.8: 2, 1.2: 3, 0.75: 4, 0.3: 5, -0.3: 6, -0.75: 7, -1.2: 8, -1.8: 9, -2.25: 10, -2.7: 11}

        return (x_coord[x[1]], y_coord[x[0]])

    def DrawCostFunction(self):
        delta = 0.1
        # dla 3D
        # delta = 0.1
        min_x = -3
        max_x = 3
        min_y = -3
        max_y = 3
        x = np.arange(min_x, max_x, delta)
        y = np.arange(min_y, max_y, delta)

        X, Y = np.meshgrid(x, y)

        pre_x = self.mpc.data.prediction(('_x', 'pos_x'))
        pre_y = self.mpc.data.prediction(('_x', 'pos_y'))

        pre_pos = []
        for j in range(len(self.bots)):
            pre_pos.append([])
            for i in range(len(pre_x[j])):
                pre_pos[j].append((pre_x[j][i][0], pre_y[j][i][0]))



        def funkcja(x, bot):
            w_lterm_dist = 80

            mterm = 0
            lterm = 0


            aim_conv = self.ConvertOnRealCoordinates(self.bots[bot].aimPosition[0], self.bots[bot].aimPosition[1])
            lterm += (w_lterm_dist * abs(x[0] - aim_conv[0]) + w_lterm_dist *abs(x[1] - aim_conv[1])) / (2*w_lterm_dist)
            #lterm += self.Norm2(x, aim_conv)

            for i in range(len(self.bots)):
                if i != bot:
                    aim_conv = self.ConvertOnRealCoordinates(self.bots[i].aimPosition[0], self.bots[i].aimPosition[1])
                    pos_conv = self.ConvertOnRealCoordinates(self.bots[i].aimPosition[0], self.bots[i].aimPosition[1])
                    lterm += w_lterm_dist * self.MyNorm((pos_conv[0],pos_conv[1]), (aim_conv[0], aim_conv[1]))
                        #        / (2 * w_lterm_dist)
                    #lterm += (w_lterm_dist * abs(pos_conv[0] - aim_conv[0]) + w_lterm_dist * abs(pos_conv[1] - aim_conv[1]))\
                    #        / (2 * w_lterm_dist)
                    #lterm += self.Norm2(pos_conv, aim_conv)

            for i in range(len(self.bots)):
                for j in range(len(self.bots)):
                    if i == j:
                        continue
                    else:
                        pass
                        # lterm += 15*self.SaturationFunction(fabs(model.x['pos_x', i] - model.x['pos_x', j]), 2)\
                        #         +15*self.SaturationFunction(fabs(model.x['pos_y', i] - model.x['pos_y', j]), 2)
                        # lterm += -15*sqrt((model.x['pos_x', i] - model.x['pos_x', j])**2 + (model.x['pos_y', i] - model.x['pos_y', j])**2)

            #lterm /= setup_mpc['n_horizon']
            lterm *= 30
            return lterm

        Z = funkcja([X, Y], 0)
        min_fun = np.min(Z)
        max_fun = np.max(Z)
        ilosc_podzialow = 30
        podzialka = (max_fun - min_fun) / ilosc_podzialow

        self.fig.clear()

        # Wykres 3D
        '''
        ax = Axes3D(self.figure)
        surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0, antialiased=False, alpha=0.5)
        ax.set_xlim([self.MinWartoscWykresuX, self.MaxWartoscWykresuX])
        ax.set_ylim([self.MinWartoscWykresuY, self.MaxWartoscWykresuY])
        ax.plot([i[0] for i in populacjaDoNarysowania], [i[1] for i in populacjaDoNarysowania], [funkcja([i[0], i[1]]) for i in populacjaDoNarysowania], "or" ,alpha= 0.7, color='black')
        ax.zaxis.set_major_locator(LinearLocator(10))
        ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))
        self.figure.colorbar(surf, shrink=0.5, aspect=5)
        ax.view_init(azim=None, elev=None)'''

        # Wykres 2D
        ax = self.fig.add_subplot()

        '''
        im = ax.imshow(Z, cmap=cm.winter,
                       origin='lower',
                       extent=[min_x, max_x, min_y, max_y],
                       norm=colors.PowerNorm(gamma=0.2))
        cset = ax.contour(Z, np.arange(min_fun, max_fun, podzialka), linewidths=2,
                          cmap=cm.winter,
                          extent=(min_x, max_x, min_y, max_y))
        ax.clabel(cset, inline=True, fmt='%1.1f', fontsize=10)
        '''

        ax.plot([self.ConvertOnRealCoordinates(i.robotPosition[0], i.robotPosition[1])[1] for i in self.bots],
                [self.ConvertOnRealCoordinates(i.robotPosition[0], i.robotPosition[1])[0] for i in self.bots], "or", color='purple')
        ax.plot([self.ConvertOnRealCoordinates(i.aimPosition[0], i.aimPosition[1])[1] for i in self.bots],
                [self.ConvertOnRealCoordinates(i.aimPosition[0], i.aimPosition[1])[0] for i in self.bots], "or", color='teal')


        for j in range(len(pre_pos)):
            for i in range(len(pre_pos[j])):
                ax.plot(pre_pos[j][i][1], pre_pos[j][i][0], "or", color='#00'+format(255-int(i*(255.0/len(pre_pos[j]))), '02x') +'00', alpha=0.8)

        points = [[1.2, 2.7], [-1.2, 2.7], [1.2, 2.25], [-1.2, 2.25], [1.2, 1.8], [-1.2, 1.8], [2.85, 1.2], [-2.85, 1.2],
                  [2.85, 0.75], [0.3, 0.75], [-0.75, 0.75], [-2.85, 0.75], [2.85, 0.3], [1.2, 0.3], [-0.75, 0.3],
                  [-2.85, 0.3], [2.85, -0.3], [0.75, -0.3],
                  [-1.2, -0.3], [-2.85, -0.3], [2.85, -0.75], [0.75, -0.75], [-0.3, -0.75], [-2.85, -0.75], [2.85, -1.2],
                  [-2.85, -1.2], [1.2, -1.8], [-1.2, -1.8], [1.2, -2.25], [-1.2, -2.25], [1.2, -2.7], [-1.2, -2.7],
                  [2.85, 1.2], [2.85, -1.2], [2.4, 1.2], [2.4, -1.2],
                  [1.8, 1.2], [1.8, -1.2], [1.2, 2.7], [1.2, -2.7], [0.75, 2.7], [0.75, 0.75], [0.75, -0.3],
                  [0.75, -2.7], [0.3, 2.7], [0.3, 0.75], [0.3, -1.2], [0.3, -2.7], [-0.3, 2.7], [-0.3, 1.2],
                  [-0.3, -0.75], [-0.3, -2.7], [-0.75, 2.7], [-0.75, 0.3],
                  [-0.75, -0.75], [-0.75, -2.7], [-1.2, 2.7], [-1.2, -2.7], [-1.8, 1.2], [-1.8, -1.2], [-2.4, 1.2],
                  [-2.4, -1.2], [-2.85, 1.2], [-2.85, -1.2]]

        for i in range(0, len(points), 2):
            ax.plot([points[i][1], points[i + 1][1]], [points[i][0], points[i + 1][0]], 'b-', alpha=0.25)

        ax.set_xlim([min_x, max_x])
        ax.set_ylim([min_y, max_y])

        ax.invert_xaxis()
        #self.fig.colorbar(im, orientation='vertical')
        #input()
        plt.pause(0.001)

    def Logger(self):
        pre_x = self.mpc.data.prediction(('_x', 'pos_x'))
        pre_y = self.mpc.data.prediction(('_x', 'pos_y'))

        pre_pos = []
        for j in range(len(self.bots)):
            pre_pos.append([])
            for i in range(len(pre_x[j])):
                pre_pos[j].append((pre_x[j][i][0], pre_y[j][i][0]))

        path_length = []
        position = []
        aim = []
        color = []
        prediction = []
        pack_in_color_zone = []
        pack_delivered = []

        pack_in_color_zone.append('red,{}'.format(self.puckInZone[ZoneColor.RED][1]+self.puckInZone[ZoneColor.RED][2]))
        pack_in_color_zone.append(
            'green,{}'.format(self.puckInZone[ZoneColor.GREEN][1] + self.puckInZone[ZoneColor.GREEN][2]))
        pack_in_color_zone.append(
            'blue,{}'.format(self.puckInZone[ZoneColor.BLUE][1] + self.puckInZone[ZoneColor.BLUE][2]))
        pack_in_color_zone.append(
            'yellow,{}'.format(self.puckInZone[ZoneColor.YELLOW][1] + self.puckInZone[ZoneColor.YELLOW][2]))

        for i in range(len(self.bots)):
            path_length.append(self.bots[i].totalPathLength)

            conv_pos = self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1])
            position.append('{},{}'.format(conv_pos[0], conv_pos[1]))

            conv_aim_pos = self.ConvertOnRealCoordinates(self.bots[i].aimPosition[0], self.bots[i].aimPosition[1])
            aim.append('{},{}'.format(conv_aim_pos[0], conv_aim_pos[1]))

            if self.bots[i].puckColor == PuckColor.RED:
                color.append('red')
            elif self.bots[i].puckColor == PuckColor.GREEN:
                color.append('green')
            elif self.bots[i].puckColor == PuckColor.BLUE:
                color.append('blue')
            elif self.bots[i].puckColor == PuckColor.YELLOW:
                color.append('yellow')
            else:
                color.append('n')

            pack_delivered.append(self.bots[i].packs_delivered)

            prediction.append([])
            for j in range(len(pre_pos[i])):
                prediction[i].append('{},{}'.format(pre_pos[i][j][0], pre_pos[i][j][1]))

        self.csv_writer.writerow([time.time() - self.start_time])
        self.csv_writer.writerow([self.planning_cycle_counter])
        self.csv_writer.writerow(path_length)
        self.csv_writer.writerow(position)
        self.csv_writer.writerow(aim)
        self.csv_writer.writerow(color)
        self.csv_writer.writerow(pack_in_color_zone)
        self.csv_writer.writerow(pack_delivered)
        for i in range(len(self.bots)):
            self.csv_writer.writerow(prediction[i])


def main():
    rospy.init_node('testBot_controller')
    file_csv = ''
    file_robots = ''
    if len(sys.argv) == 3:
        file_robots = sys.argv[1]
        file_csv = sys.argv[2]

    elif len(sys.argv) == 2:
        file_robots = sys.argv[1]

    controller = Controller(file_robots, file_csv)
    while not rospy.is_shutdown():
        controller.mianLoop()
        rospy.sleep(0.01)


if __name__ == '__main__':
    main()
