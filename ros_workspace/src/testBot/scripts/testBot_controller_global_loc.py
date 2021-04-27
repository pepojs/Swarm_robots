#!/usr/bin/python3

import json
import sys
import enum
import math
import numpy as np
import heapq
import csv
import time
import xml.etree.ElementTree as ET

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

from testBot_robot import ZoneColor
from testBot_robot import PuckColor
from testBot_robot import OrientationFromTag

import matplotlib.pyplot as plt
import matplotlib.cm as cm



class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self) -> bool:
        return not self.elements

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

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
        self.aimPosition = position
        self.puckColor = PuckColor.NONPUCK
        self.robotInZone = False
        self.zoneColor = ZoneColor.BLACK
        self.zoneNumber = 0
        self.globalOrientation = orientation
        self.finishPosition = finishPosition
        self.backToFinishPosition = False
        self.path = []
        self.waitTime = 0
        self.totalPathLength = 0
        self.packs_delivered = 0
        self.next_planning = False

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

        self.bots = []
        self.botsFinished = []
        self.file_robots_name = file_robots

        self.planning_cycle_counter = 0

        if file_robots == '':
            self.bots.append(RobotParameters('bot1', (3, 6), -math.pi/2, (3, 6), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot2', (5, 3), math.pi, (5, 3), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot3', (8, 5), math.pi/2, (8, 5), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot4', (6, 8), 0, (6, 8), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot5', (2, 5), -math.pi/2, (2, 5), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot6', (6, 2), math.pi, (6, 2), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot7', (9, 6), math.pi/2, (9, 6), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot8', (5, 9), 0, (5, 9), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot9', (0, 7), -math.pi/2, (0, 7), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot10', (4, 0), math.pi, (4, 0), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot11', (11, 4), math.pi/2, (11, 4), self.callbackRobotAnswerBot))
            self.bots.append(RobotParameters('bot12', (7, 11), 0, (7, 11), self.callbackRobotAnswerBot))
            self.botsFinished = [False, False, False, False, False, False, False, False, False, False, False, False]

        else:
            self.readRobotsFromFile()

        self.SetEnterZonePositionLikeMPC()
        self.SetPuckInZone()
        self.GenerateStartMapLikeMPC()
        self.GenerateStartPathMap()

        self.pathWasChange = True

        if file_csv == '':
            self.file_log_name = 'controller_log.csv'
        else:
            self.file_log_name = file_csv

        self.file_log_csv = open(self.file_log_name, 'w')
        self.csv_writer = csv.writer(self.file_log_csv, delimiter=';')
        self.csv_writer.writerow([len(self.bots), 'p'])

        '''
        plt.ion()
        self.fig = plt.figure()
        plt.xticks(np.arange(0, 12, 1))
        plt.yticks(np.arange(0, 12, 1))
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

        finished = True
        for i in self.botsFinished:
            if not i:
                finished = False
                break

        if finished:
            self.Logger()
            self.csv_writer.writerow(['#finish'])
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
                if len(self.bots[robotNumber-1].path) == 0:
                    self.bots[robotNumber-1].path = self.SearchNearestPathToPosition(self.bots[robotNumber-1].finishPosition, robotNumber, self.GenerateCostMapForRobot(robotNumber))
                    self.AddRobotPathToMap(robotNumber)
                    self.bots[robotNumber-1].robotState = RobotState.READY

        elif robotState == RobotState.IDLE:
            self.Logger()
            if self.bots[robotNumber-1].puckColor == PuckColor.NONPUCK:
                self.bots[robotNumber - 1].path = self.SearchPathToNearestZone(ZoneColor.BLACK, robotNumber, self.GenerateCostMapForRobot(robotNumber))
                self.AddRobotPathToMap(robotNumber)
                self.bots[robotNumber - 1].zoneColor = ZoneColor.BLACK

                if len(self.bots[robotNumber - 1].path) == 1:
                    self.bots[robotNumber - 1].robotState = RobotState.ENTER_ZONE
                    #blokada strefy
                    self.bots[robotNumber-1].sendCommand(robotNumber, str(RobotCommand.ENTER_TO_ZONE), [self.bots[robotNumber-1].zoneColor])

                else:
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
            if len(self.bots[robotNumber - 1].path) > 1:
                self.RemoveRobotPathFromPathMap(robotNumber)
                if self.bots[robotNumber - 1].backToFinishPosition:
                    self.bots[robotNumber - 1].path = self.SearchNearestPathToPosition(self.bots[robotNumber - 1].finishPosition, robotNumber,
                        self.GenerateCostMapForRobot(robotNumber))
                else:
                    self.bots[robotNumber - 1].path = self.SearchPathToNearestZone(self.bots[robotNumber - 1].zoneColor, robotNumber,
                                                                           self.GenerateCostMapForRobot(robotNumber))
                self.AddRobotPathToMap(robotNumber)

            path = self.bots[robotNumber - 1].path
            if len(path) > 1:
                self.MakeOneStepInPath(robotNumber)

            elif len(path) == 1 and not self.bots[robotNumber - 1].backToFinishPosition:
                self.bots[robotNumber - 1].robotState = RobotState.ENTER_ZONE
                self.bots[robotNumber - 1].sendCommand(robotNumber, str(RobotCommand.ENTER_TO_ZONE),
                                                       [self.bots[robotNumber - 1].zoneColor])

            elif len(path) == 1 and self.bots[robotNumber - 1].backToFinishPosition:
                self.bots[robotNumber - 1].robotState = RobotState.FINISH

            elif len(path) == 0:
                self.bots[robotNumber - 1].robotState = RobotState.FINISH
                self.bots[robotNumber - 1].backToFinishPosition = True

        elif robotState == RobotState.WAIT:
            self.MakeOneStepInPath(robotNumber)

    def callbackRobotAnswerBot(self, msg):
        robotNumber, answer, answerParam = decode_command(msg.data)
        print("Robot: {}, answer {}, parma: {}".format(self.bots[robotNumber-1].robotName, answer, answerParam))


        if answer == str(RobotCommand.MOVE_TO_POSITION):
            if len(answerParam) == 3:
                self.bots[robotNumber-1].globalOrientation = answerParam[2]
                lastPos = self.bots[robotNumber-1].robotPosition
                currentPos = self.bots[robotNumber-1].path[1]

                self.bots[robotNumber - 1].totalPathLength += self.TaxicabNorm(currentPos, lastPos)

                #if not (currentPos in self.listZoneEnterPosition):
                self.robotMap[lastPos[0]][lastPos[1]] = 0

                self.bots[robotNumber-1].path.remove(lastPos)
                self.RemoveOneStepFromPathMap(robotNumber, lastPos)
                self.bots[robotNumber-1].robotPosition = currentPos

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

                self.bots[robotNumber-1].puckColor = answerParam[0]

                self.bots[robotNumber-1].robotState = RobotState.ESCAPE_ZONE
                self.bots[robotNumber-1].sendCommand(robotNumber, str(RobotCommand.ESCAPE_ZONE), [])

        elif answer == str(RobotCommand.ESCAPE_ZONE):
            if len(answerParam) == 3:
                self.bots[robotNumber-1].globalOrientation = answerParam[2]
                self.bots[robotNumber-1].robotInZone = False
                if self.bots[robotNumber-1].puckColor == PuckColor.NONPUCK:
                    self.bots[robotNumber-1].path = self.SearchPathToNearestZone(ZoneColor.BLACK, robotNumber, self.GenerateCostMapForRobot(robotNumber))
                    self.AddRobotPathToMap(robotNumber)
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

                    self.bots[robotNumber-1].path = self.SearchPathToNearestZone(self.bots[robotNumber-1].zoneColor, robotNumber,
                                                                     self.GenerateCostMapForRobot(robotNumber))
                    self.AddRobotPathToMap(robotNumber)

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

        j = 1
        for i in self.bots:
            self.robotMap[i.robotPosition[0]][i.robotPosition[1]] = j
            if i.robotPosition in self.listZoneEnterPosition:
                neighbors = self.GetNeighborsLikeMPC(i.robotPosition)
                '''
                if len(neighbors) == 1:
                    neighbor = neighbors[0]
                    dx = i.robotPosition[0] - neighbor[0]
                    dy = i.robotPosition[1] - neighbor[1]
                    self.robotMap[neighbor[0]][neighbor[1]] = j
                    self.robotMap[neighbor[0] - dx][neighbor[1] - dy] = j
                else:
                '''
                for k in neighbors:
                    self.robotMap[k[0]][k[1]] = j
            j = j + 1

    def GenerateStartMapLikeMPC(self):
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

        # MPC
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

        j = 1
        for i in self.bots:
            self.robotMap[i.robotPosition[0]][i.robotPosition[1]] = j
            '''
            if i.robotPosition in self.listZoneEnterPosition:
                neighbors = self.GetNeighborsLikeMPC(i.robotPosition)
                
                for k in neighbors:
                    self.robotMap[k[0]][k[1]] = j
            '''
            j = j + 1

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

    def SetEnterZonePositionLikeMPC(self):
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

    def GenerateCostMapForRobot(self, robotNumber):
        costMap = []
        for i in range (len(self.robotMap)):
            costMap.append([])
            for j in range(len(self.robotMap[i])):
                if self.robotMap[i][j] == -1:
                    costMap[i].append(math.inf)

                elif self.robotMap[i][j] != 0 and self.robotMap[i][j] != robotNumber:
                    costMap[i].append(7)

                elif self.robotMap[i][j] == robotNumber:
                    costMap[i].append(0)

                else:
                    costMap[i].append(1)

        #Dodac koszty przeciecia sie tras
        for i in range(len(self.robotPathMap)):
            for j in range(len(self.robotPathMap[i])):
                tempLenList = len(self.robotPathMap[i][j])
                if tempLenList > 0:
                    if robotNumber in self.robotPathMap[i][j]:
                        costMap[i][j] += (tempLenList - 1)
                    else:
                        costMap[i][j] += tempLenList

        for i in range(len(self.bots)):
            if i != (robotNumber - 1):
                if len(self.bots[i].path) >= 3:
                    firstStep = self.bots[i].path[1]
                    secondStep = self.bots[i].path[2]
                    costMap[firstStep[0]][firstStep[1]] += 5
                    costMap[secondStep[0]][secondStep[1]] += 3

                elif len(self.bots[i].path) == 2:
                    firstStep = self.bots[i].path[1]
                    costMap[firstStep[0]][firstStep[1]] += 5

        return costMap

    def SearchPathToNearestZone(self, zoneColor: ZoneColor, robotNumber, costMap):
        if self.bots[robotNumber - 1].next_planning:
            for i in range(len(self.bots)):
                self.bots[i].next_planning = False

            self.planning_cycle_counter += 1

        path = []
        cost = math.inf
        if zoneColor == ZoneColor.BLACK:
            for zoneNumber in range(1,5):
                if self.puckInZone[zoneColor][zoneNumber] == 1:
                    tempPath, tempCost = self.SearchPathToZone(zoneColor, zoneNumber, robotNumber, costMap)
                    #print("Path to black zone number: {}, cost: {}".format(zoneNumber, tempCost))
                    if tempCost < cost:
                        path = tempPath
                        cost = tempCost

        else:
            for zoneNumber in range(1, 3):
                if self.puckInZone[zoneColor][zoneNumber] <= 24:
                    tempPath, tempCost = self.SearchPathToZone(zoneColor, zoneNumber, robotNumber, costMap)
                    #print("Path to color zone number: {}, cost: {}".format(zoneNumber, tempCost))
                    if tempCost < cost:
                        path = tempPath
                        cost = tempCost

        #print("Path: ", path)
        if len(path) > 0:
            self.bots[robotNumber - 1].aimPosition = path[len(path)-1]
        else:
            self.bots[robotNumber - 1].aimPosition = self.bots[robotNumber - 1].robotPosition

        self.bots[robotNumber - 1].next_planning = True
        return path

    def SearchPathToZone(self, zoneColor: ZoneColor, zoneNumber, robotNumber, costMap):
        goalPos = self.zoneEnterPosition[zoneColor][zoneNumber]

        frontier = PriorityQueue()
        frontier.put(self.bots[robotNumber-1].robotPosition, 0)
        cameFrom = dict()
        currentCost = dict()

        cameFrom[self.bots[robotNumber-1].robotPosition] = None
        currentCost[self.bots[robotNumber-1].robotPosition] = 0

        #print('GoalPos: {}'.format(goalPos))

        while not frontier.empty():
            currentPos = frontier.get()

            #print('CurrentPos: {}'.format(currentPos))

            if currentPos == goalPos:
                break

            #print('Neighbors {}'.format(self.GetNeighbors(currentPos)))

            for next in self.GetNeighborsLikeMPC(currentPos):
                new_cost = currentCost[currentPos] + costMap[next[0]][next[1]]

                #print('try next pos {}, currentCost {}, costMap {}'. format(next, currentCost[currentPos], costMap[next[0]][next[1]]))

                if next not in currentCost or new_cost < currentCost[next]:
                    currentCost[next] = new_cost
                    priority = new_cost + self.TaxicabNorm(next, goalPos)

                    #print('Next: {}, Goal: {} -> TaxiNorm: {}, prio: {}'.format(next, goalPos, self.TaxicabNorm(next, goalPos), priority))

                    frontier.put(next, priority)
                    cameFrom[next] = currentPos

        current = goalPos
        path = []
        while current != self.bots[robotNumber-1].robotPosition:
            path.append(current)
            current = cameFrom[current]

        path.append(self.bots[robotNumber-1].robotPosition)
        path.reverse()

        return path, currentCost[goalPos]

    def SearchNearestPathToPosition(self, position, robotNumber, costMap):
        if self.bots[robotNumber - 1].next_planning:
            for i in range(len(self.bots)):
                self.bots[i].next_planning = False

            self.planning_cycle_counter += 1

        goalPos = position

        frontier = PriorityQueue()
        frontier.put(self.bots[robotNumber - 1].robotPosition, 0)
        cameFrom = dict()
        currentCost = dict()

        cameFrom[self.bots[robotNumber - 1].robotPosition] = None
        currentCost[self.bots[robotNumber - 1].robotPosition] = 0

        while not frontier.empty():
            currentPos = frontier.get()

            if currentPos == goalPos:
                break

            for next in self.GetNeighborsLikeMPC(currentPos):
                new_cost = currentCost[currentPos] + costMap[next[0]][next[1]]
                if next not in currentCost or new_cost < currentCost[next]:
                    currentCost[next] = new_cost
                    priority = new_cost + self.TaxicabNorm(next, goalPos)
                    frontier.put(next, priority)
                    cameFrom[next] = currentPos

        current = goalPos
        path = []
        while current != self.bots[robotNumber - 1].robotPosition:
            path.append(current)
            current = cameFrom[current]

        path.append(self.bots[robotNumber - 1].robotPosition)
        path.reverse()

        if len(path) > 0:
            self.bots[robotNumber - 1].aimPosition = path[len(path)-1]
        else:
            self.bots[robotNumber - 1].aimPosition = self.bots[robotNumber - 1].robotPosition

        print("Path: ", path)

        self.bots[robotNumber - 1].next_planning = True
        return path

    def TaxicabNorm(self, position1, position2):
        return abs(position2[0] - position1[0]) + abs(position2[1] - position1[1])

    def Norm2(self, position1, position2):
        return math.sqrt((position2[0] - position1[0])**2 + (position2[1] - position1[1])**2)

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
            if self.robotMap[i[0]][i[1]] == -1:
                toRemove.append(i)

        for i in toRemove:
            result.remove(i)

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

        return result

    def GetNeighborsLikeMPC(self, position):
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
            if self.robotMap[i[0]][i[1]] == -1:
                toRemove.append(i)

        for i in toRemove:
            if i in result:
                result.remove(i)


        return result

    def MakeOneStepInPath(self, robotNumber):
        robotState = self.bots[robotNumber - 1].robotState
        if len(self.bots[robotNumber - 1].path) >= 2 and (robotState == RobotState.READY or robotState == RobotState.WAIT):

            nextPos = self.bots[robotNumber - 1].path[1]

            if self.robotMap[nextPos[0]][nextPos[1]] == 0 or self.robotMap[nextPos[0]][nextPos[1]] == robotNumber:
                self.bots[robotNumber - 1].waitTime = 0
                self.robotMap[nextPos[0]][nextPos[1]] = robotNumber
                self.bots[robotNumber - 1].robotState = RobotState.FORWARD
                x,y = self.ConvertOnRealCoordinates(nextPos[0], nextPos[1])
                self.bots[robotNumber - 1].sendCommand(robotNumber, str(RobotCommand.MOVE_TO_POSITION), [x,y])
                self.Logger()

            else:
                self.bots[robotNumber - 1].robotState = RobotState.WAIT
                self.bots[robotNumber - 1].waitTime += 1

                if self.bots[robotNumber - 1].waitTime == 100:
                    #print('Wait time finish')
                    self.RemoveRobotPathFromPathMap(robotNumber)
                    if self.bots[robotNumber - 1].backToFinishPosition:
                        self.bots[robotNumber - 1].path = self.SearchNearestPathToPosition(
                            self.bots[robotNumber - 1].finishPosition, robotNumber,
                            self.GenerateCostMapForRobot(robotNumber))
                    else:
                        self.bots[robotNumber - 1].path = self.SearchPathToNearestZone(
                            self.bots[robotNumber - 1].zoneColor, robotNumber,
                            self.GenerateCostMapForRobot(robotNumber))
                    self.AddRobotPathToMap(robotNumber)

                if self.bots[robotNumber - 1].waitTime == 800:
                    #print('Wait time finish')
                    self.RemoveRobotPathFromPathMap(robotNumber)
                    if self.bots[robotNumber - 1].backToFinishPosition:
                        self.bots[robotNumber - 1].path = self.SearchNearestPathToPosition(
                            self.bots[robotNumber - 1].finishPosition, robotNumber,
                            self.GenerateCostMapForRobotWithScale(robotNumber, 2))
                    else:
                        self.bots[robotNumber - 1].path = self.SearchPathToNearestZone(
                            self.bots[robotNumber - 1].zoneColor, robotNumber,
                            self.GenerateCostMapForRobotWithScale(robotNumber, 2))
                    self.AddRobotPathToMap(robotNumber)

                if self.bots[robotNumber - 1].waitTime == 1600:
                    #print('Wait time finish')
                    self.bots[robotNumber - 1].waitTime = 0
                    self.RemoveRobotPathFromPathMap(robotNumber)
                    if self.bots[robotNumber - 1].backToFinishPosition:
                        self.bots[robotNumber - 1].path = self.SearchNearestPathToPosition(
                            self.bots[robotNumber - 1].finishPosition, robotNumber,
                            self.GenerateCostMapForRobotWithScale(robotNumber, 6))
                    else:
                        self.bots[robotNumber - 1].path = self.SearchPathToNearestZone(
                            self.bots[robotNumber - 1].zoneColor, robotNumber,
                            self.GenerateCostMapForRobotWithScale(robotNumber, 6))
                    self.AddRobotPathToMap(robotNumber)
                    
                if len(self.bots[robotNumber - 1].path) == 0 :
                    self.bots[robotNumber - 1].robotState = RobotState.FINISH
                    self.bots[robotNumber - 1].backToFinishPosition = True

                return

    def GenerateCostMapForRobotWithScale(self, robotNumber, scale):
        costMap = []
        for i in range(len(self.robotMap)):
            costMap.append([])
            for j in range(len(self.robotMap[i])):
                if self.robotMap[i][j] == -1:
                    costMap[i].append(math.inf)

                elif self.robotMap[i][j] != 0 and self.robotMap[i][j] != robotNumber:
                    costMap[i].append(7*scale)

                elif self.robotMap[i][j] == robotNumber:
                    costMap[i].append(0)

                else:
                    costMap[i].append(1)

        # Dodac koszty przeciecia sie tras
        for i in range(len(self.robotPathMap)):
            for j in range(len(self.robotPathMap[i])):
                tempLenList = len(self.robotPathMap[i][j])
                if tempLenList > 0:
                    if robotNumber in self.robotPathMap[i][j]:
                        costMap[i][j] += (tempLenList - 1)
                    else:
                        costMap[i][j] += tempLenList

        for i in range(len(self.bots)):
            if i != (robotNumber - 1):
                if len(self.bots[i].path) >= 3:
                    firstStep = self.bots[i].path[1]
                    secondStep = self.bots[i].path[2]
                    costMap[firstStep[0]][firstStep[1]] += 5
                    costMap[secondStep[0]][secondStep[1]] += 3

                elif len(self.bots[i].path) == 2:
                    firstStep = self.bots[i].path[1]
                    costMap[firstStep[0]][firstStep[1]] += 5

        return costMap

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

    def Logger(self):
        path_length = []
        position = []
        aim = []
        color = []
        path = []
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

            path.append([])
            for j in range(len(self.bots[i].path)):
                conv_pos = self.ConvertOnRealCoordinates(self.bots[i].path[j][0], self.bots[i].path[j][1])
                path[i].append('{},{}'.format(conv_pos[0], conv_pos[1]))

        self.csv_writer.writerow([time.time() - self.start_time])
        self.csv_writer.writerow([self.planning_cycle_counter])
        self.csv_writer.writerow(path_length)
        self.csv_writer.writerow(position)
        self.csv_writer.writerow(aim)
        self.csv_writer.writerow(color)
        self.csv_writer.writerow(pack_in_color_zone)
        self.csv_writer.writerow(pack_delivered)
        for i in range(len(self.bots)):
            self.csv_writer.writerow(path[i])

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
