#!/usr/bin/python3

import json
import sys
import enum
import math
import numpy as np
import heapq

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
        self.mpc = 0

        self.pubController = rospy.Publisher('/' + robotName + '/robotController', String, queue_size=10)
        self.pubRobotStop = rospy.Publisher("/" + robotName + "/robotStop", Bool, queue_size=10)
        rospy.Subscriber("/" + robotName + "/robotAnswer", String, callbackRobotAnswer)

    def sendCommand(self, robotNumber, commandName : String, paramList : list):
        self.pubController.publish(code_command(robotNumber, commandName, paramList))

class Controller:
    def __init__(self):
        self.puckInZone = dict()
        self.zoneEnterPosition = dict()
        self.listZoneEnterPosition = list()

        self.bots = [RobotParameters('bot1', (3, 6), -math.pi/2, (3, 6), self.callbackRobotAnswerBot)]
        #self.bots.append(RobotParameters('bot2', (0.525, 0.3), math.pi, (0.525, 0.3), self.callbackRobotAnswerBot))
        '''
        self.bots.append(RobotParameters('bot3', (7, 5), math.pi/2, (7, 5), self.callbackRobotAnswerBot))
        self.bots.append(RobotParameters('bot4', (6, 7), 0, (6, 7), self.callbackRobotAnswerBot))
        self.bots.append(RobotParameters('bot5', (2, 5), -math.pi/2, (2, 5), self.callbackRobotAnswerBot))
        self.bots.append(RobotParameters('bot6', (6, 2), math.pi, (6, 2), self.callbackRobotAnswerBot))
        self.bots.append(RobotParameters('bot7', (9, 6), math.pi/2, (9, 6), self.callbackRobotAnswerBot))
        self.bots.append(RobotParameters('bot8', (5, 9), 0, (5, 9), self.callbackRobotAnswerBot))
        self.bots.append(RobotParameters('bot9', (0, 7), -math.pi/2, (0, 7), self.callbackRobotAnswerBot))
        self.bots.append(RobotParameters('bot10', (4, 0), math.pi, (4, 0), self.callbackRobotAnswerBot))
        self.bots.append(RobotParameters('bot11', (11, 4), math.pi/2, (11, 4), self.callbackRobotAnswerBot))
        self.bots.append(RobotParameters('bot12', (7, 11), 0, (7, 11), self.callbackRobotAnswerBot))
        '''
        self.botsFinished = [False]#, False, False, False, False, False, False, False, False, False, False, False]
        self.SetEnterZonePositionMPC()
        self.SetPuckInZone()
        model = self.robotKinematicModel()
        self.setupAllMPC(model)
        self.GenerateStartMap()
        #self.GenerateStartPathMap()

        #self.pathWasChange = True
        print()

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

    def mianLoop(self):

        for i in range(len(self.bots)):
            self.updateRobot(i+1)

        finished = True
        for i in self.botsFinished:
            if not i:
                finished = False
                break

        if finished:
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
            print('aim: ', self.bots[robotNumber - 1].aimPosition)
            print('pos: ', self.bots[robotNumber - 1].robotPosition)
            if self.bots[robotNumber - 1].aimPosition != self.bots[robotNumber - 1].robotPosition:
                self.MakeOneStep(robotNumber)

            else:

                if not self.bots[robotNumber - 1].backToFinishPosition:
                    self.bots[robotNumber - 1].robotState = RobotState.ENTER_ZONE
                    self.bots[robotNumber - 1].sendCommand(robotNumber, str(RobotCommand.ENTER_TO_ZONE),
                                                           [self.bots[robotNumber - 1].zoneColor])

                elif self.bots[robotNumber - 1].backToFinishPosition:
                    self.bots[robotNumber - 1].robotState = RobotState.FINISH


        elif robotState == RobotState.WAIT:
            self.MakeOneStep(robotNumber)

    def callbackRobotAnswerBot(self, msg):
        robotNumber, answer, answerParam = decode_command(msg.data)
        print("Robot: {}, answer {}, parma: {}".format(self.bots[robotNumber-1].robotName, answer, answerParam))


        if answer == str(RobotCommand.MOVE_TO_POSITION):
            if len(answerParam) == 3:
                self.bots[robotNumber-1].globalOrientation = answerParam[2]
                self.bots[robotNumber-1].robotState = RobotState.READY

        elif answer == str(RobotCommand.MOVE_WITH_ORIENTATION):
            if len(answerParam) == 3:
                self.bots[robotNumber-1].globalOrientation = answerParam[2]
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
        robotNumber = 1
        temp = self.bots[robotNumber-1].mpc.get_tvp_template()
        aim_conv = self.ConvertOnRealCoordinates(self.bots[robotNumber-1].aimPosition[0], self.bots[robotNumber-1].aimPosition[1])
        temp_list = [aim_conv[0], aim_conv[1]]

        temp_robot_coor = []
        for i in range(len(self.bots)):
            temp_robot_coor.append(self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1]))

        for i in range(len(self.bots)):
            if i == robotNumber-1:
                continue
            else:
                temp_list.append(temp_robot_coor[i][0])

        for i in range(len(self.bots)):
            if i == robotNumber - 1:
                continue
            else:
                temp_list.append(temp_robot_coor[i][1])

        temp['_tvp', :] = np.array(temp_list)
        return temp

    def tvp_fun2(self, t_now):
        robotNumber = 2
        temp = self.bots[robotNumber - 1].mpc.get_tvp_template()
        aim_conv = self.ConvertOnRealCoordinates(self.bots[robotNumber - 1].aimPosition[0],
                                                 self.bots[robotNumber - 1].aimPosition[1])
        temp_list = [aim_conv[0], aim_conv[1]]

        temp_robot_coor = []
        for i in range(len(self.bots)):
            temp_robot_coor.append(
                self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1]))

        for i in range(len(self.bots)):
            if i == robotNumber - 1:
                continue
            else:
                temp_list.append(temp_robot_coor[i][0])

        for i in range(len(self.bots)):
            if i == robotNumber - 1:
                continue
            else:
                temp_list.append(temp_robot_coor[i][1])

        temp['_tvp', :] = np.array(temp_list)
        return temp

    def tvp_fun3(self, t_now):
        robotNumber = 3
        temp = self.bots[robotNumber - 1].mpc.get_tvp_template()
        aim_conv = self.ConvertOnRealCoordinates(self.bots[robotNumber - 1].aimPosition[0],
                                                 self.bots[robotNumber - 1].aimPosition[1])
        temp_list = [aim_conv[0], aim_conv[1]]

        temp_robot_coor = []
        for i in range(len(self.bots)):
            temp_robot_coor.append(
                self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1]))

        for i in range(len(self.bots)):
            if i == robotNumber - 1:
                continue
            else:
                temp_list.append(temp_robot_coor[i][0])

        for i in range(len(self.bots)):
            if i == robotNumber - 1:
                continue
            else:
                temp_list.append(temp_robot_coor[i][1])

        temp['_tvp', :] = np.array(temp_list)
        return temp

    def tvp_fun4(self, t_now):
        robotNumber = 4
        temp = self.bots[robotNumber - 1].mpc.get_tvp_template()
        aim_conv = self.ConvertOnRealCoordinates(self.bots[robotNumber - 1].aimPosition[0],
                                                 self.bots[robotNumber - 1].aimPosition[1])
        temp_list = [aim_conv[0], aim_conv[1]]

        temp_robot_coor = []
        for i in range(len(self.bots)):
            temp_robot_coor.append(
                self.ConvertOnRealCoordinates(self.bots[i].robotPosition[0], self.bots[i].robotPosition[1]))

        for i in range(len(self.bots)):
            if i == robotNumber - 1:
                continue
            else:
                temp_list.append(temp_robot_coor[i][0])

        for i in range(len(self.bots)):
            if i == robotNumber - 1:
                continue
            else:
                temp_list.append(temp_robot_coor[i][1])

        temp['_tvp', :] = np.array(temp_list)
        return temp

    def dist_to_line(self, x, y, A, B, C):
        return fabs(A * x + B * y + C) / (sqrt(A ** 2 + B ** 2))

    def dist_to_section(self, x, y, point1, point2):
        t = ((point2[0] - point1[0]) * (x - point1[0]) + (point2[1] - point1[1]) * (y - point1[1])) / \
            ((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)
        t = fmax(0, fmin(1, t))
        projection = (point1[0] + t * (point2[0] - point1[0]), point1[1] + t * (point2[1] - point1[1]))
        dist = if_else(lt(fabs(x - projection[0]), 0.0001)*lt(fabs(y - projection[1]), 0.0001), 0.0001, self.Norm2([x, y], projection))
        return dist#self.Norm2([x, y], projection)

    def function(self, x, y):
        temp_min = self.dist_to_section(x, y, [1.2, 2.7], [-1.2, 2.7]) #pionowa 2.7
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [1.2, 2.25], [-1.2, 2.25])) #pionowa 2.25
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [1.2, 1.8], [-1.2, 1.8])) #pionowa 1.8
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.7, 1.2], [-2.7, 1.2]))  # pionowa 1.2
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.7, 0.75], [0.3, 0.75]))  # pionowa gorna 0.75
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-0.75, 0.75], [-2.7, 0.75]))  # pionowa dolna 0.75
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.7, 0.3], [1.2, 0.3]))  # pionowa gorna 0.3
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-0.75, 0.3], [-2.7, 0.3]))  # pionowa dolna 0.3
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.7, -0.3], [0.75, -0.3]))  # pionowa gorna -0.3
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-1.2, -0.3], [-2.7, -0.3]))  # pionowa dolna -0.3
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.7, -0.75], [0.75, -0.75]))  # pionowa gorna -0.75
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-0.3, -0.75], [-2.7, -0.75]))  # pionowa dolna -0.75
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.7, -1.2], [-2.7, -1.2]))  # pionowa -1.2
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [1.2, -1.8], [-1.2, -1.8]))  # pionowa -1.8
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [1.2, -2.25], [-1.2, -2.25]))  # pionowa -2.25
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [1.2, -2.7], [-1.2, -2.7]))  # pionowa -2.7

        temp_min = fmin(temp_min, self.dist_to_section(x, y, [2.7, 1.2], [2.7, -1.2]))  # pozioma 2.7
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
        temp_min = fmin(temp_min, self.dist_to_section(x, y, [-2.7, 1.2], [-2.7, -1.2]))  # pozioma -2.7

        return temp_min

    def robotKinematicModel(self):
        model_type = 'continuous'
        model = do_mpc.model.Model(model_type)
        pos_x = model.set_variable('_x', 'pos_x')
        pos_y = model.set_variable('_x', 'pos_y')
        theta = model.set_variable('_x', 'theta')

        v = model.set_variable('_u', 'v')
        omega = model.set_variable('_u', 'omega')

        aim_x = model.set_variable('_tvp', 'aim_x')
        aim_y = model.set_variable('_tvp', 'aim_y')

        model.set_variable('_tvp', 'avoid_x', shape=(len(self.bots)-1, 1))
        model.set_variable('_tvp', 'avoid_y', shape=(len(self.bots) - 1, 1))


        # model.set_rhs('pos_x', v * cos(omega))
        # model.set_rhs('pos_y', v * sin(omega))

        model.set_rhs('pos_x', v * cos(theta))
        model.set_rhs('pos_y', v * sin(theta))
        model.set_rhs('theta', omega)

        model.set_expression('grid', self.function(pos_x, pos_y))

        model.setup()

        return model

    def setupAllMPC(self, model):


        setup_mpc = {
            'n_horizon': 12,
            't_step': 1,
            'n_robust': 0,
            'store_full_solution': True,
            'nlpsol_opts': {'ipopt.max_iter': 700, 'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': 0}
            # 'ipopt.max_iter':500
        }

        tvp_fun_list = [self.tvp_fun1, self.tvp_fun2, self.tvp_fun3, self.tvp_fun4]
        i = 0

        for bot in self.bots:
            bot.mpc = do_mpc.controller.MPC(model)

            bot.mpc.set_param(**setup_mpc)
            bot.mpc.set_tvp_fun(tvp_fun_list[i])

            mterm = SX(0)#10*fabs(model.x['pos_x'] - model.tvp['aim_x']) + 10*fabs(model.x['pos_y'] - model.tvp['aim_y'])
            lterm = 300*fabs(model.x['pos_x'] - model.tvp['aim_x']) + 300*fabs(model.x['pos_y'] - model.tvp['aim_y'])

            '''
            for j in range(len(self.bots)-1):
                mterm += -10*sqrt((model.x['pos_x'] - model.tvp['avoid_x', j])**2 + (model.x['pos_y'] - model.tvp['avoid_y', j])**2)
                lterm += -5*sqrt((model.x['pos_x'] - model.tvp['avoid_x', j])**2 + (model.x['pos_y'] - model.tvp['avoid_y', j])**2)
            '''

            bot.mpc.set_objective(mterm=mterm, lterm=lterm)
            bot.mpc.set_rterm(v=0.5, omega=3)

            bot.mpc.bounds['lower', '_x', 'pos_x'] = -3
            bot.mpc.bounds['lower', '_x', 'pos_y'] = -3
            bot.mpc.bounds['lower', '_x', 'theta'] = -np.pi

            bot.mpc.bounds['upper', '_x', 'pos_x'] = 3
            bot.mpc.bounds['upper', '_x', 'pos_y'] = 3
            bot.mpc.bounds['upper', '_x', 'theta'] = np.pi

            bot.mpc.bounds['lower', '_u', 'v'] = 0
            bot.mpc.bounds['lower', '_u', 'omega'] = -2
            #bot.mpc.bounds['lower', '_u', 'omega'] = -np.pi

            bot.mpc.bounds['upper', '_u', 'v'] = 0.5
            bot.mpc.bounds['upper', '_u', 'omega'] = 2
            #bot.mpc.bounds['lower', '_u', 'omega'] = np.pi

            bot.mpc.set_nl_cons('grid_shape', model.aux['grid'], 0.03)

            bot.mpc.setup()
            conv_pos = self.ConvertOnRealCoordinates(bot.robotPosition[0], bot.robotPosition[1])
            x0 = np.array([conv_pos[0], conv_pos[1], bot.globalOrientation]).reshape(-1, 1)

            bot.mpc.x0 = x0
            bot.mpc.set_initial_guess()

            i += 1

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

        if zoneColor == ZoneColor.BLACK:
            for zoneNumber in range(1,5):
                if self.puckInZone[zoneColor][zoneNumber] == 1:
                    temp_enter = self.zoneEnterPosition[zoneColor][zoneNumber]
                    tempDist = self.TaxicabNorm(self.bots[robotNumber-1].robotPosition, temp_enter)
                    print(tempDist)
                    if tempDist < dist:
                        dist = tempDist
                        aim = temp_enter

        else:
            for zoneNumber in range(1, 3):
                if self.puckInZone[zoneColor][zoneNumber] <= 24:
                    temp_enter = self.zoneEnterPosition[zoneColor][zoneNumber]
                    tempDist = self.TaxicabNorm(self.bots[robotNumber - 1].robotPosition, temp_enter)
                    if tempDist < dist:
                        dist = tempDist
                        aim = temp_enter

        if zoneColor == ZoneColor.BLACK and np.isinf(dist):
            self.bots[robotNumber - 1].aimPosition = self.bots[robotNumber-1].finishPosition
            self.bots[robotNumber - 1].backToFinishPosition = True
            conv_pos = self.ConvertOnRealCoordinates(self.bots[robotNumber-1].robotPosition[0], self.bots[robotNumber-1].robotPosition[1])
            x0 = np.array([conv_pos[0], conv_pos[1], self.bots[robotNumber-1].globalOrientation]).reshape(-1, 1)

            self.bots[robotNumber - 1].mpc.x0 = x0
            self.bots[robotNumber - 1].mpc.set_initial_guess()
            return

        print(aim)
        self.bots[robotNumber-1].aimPosition = tuple(aim)
        conv_pos = self.ConvertOnRealCoordinates(self.bots[robotNumber - 1].robotPosition[0], self.bots[robotNumber - 1].robotPosition[1])
        x0 = np.array([conv_pos[0], conv_pos[1], self.bots[robotNumber - 1].globalOrientation]).reshape(-1, 1)

        self.bots[robotNumber - 1].mpc.x0 = x0
        self.bots[robotNumber - 1].mpc.set_initial_guess()

    def TaxicabNorm(self, position1, position2):
        return abs(position2[0] - position1[0]) + abs(position2[1] - position1[1])

    def Norm2(self, position1, position2):
        return sqrt((position2[0] - position1[0])**2 + (position2[1] - position1[1])**2)

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
            if self.robotMap[i[0]][i[1]] == -1:
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

    def MakeOneStep(self, robotNumber):
        robotState = self.bots[robotNumber - 1].robotState
        if robotState == RobotState.READY or robotState == RobotState.WAIT:

            self.bots[robotNumber - 1].robotState = RobotState.FORWARD
            conv_pos = self.ConvertOnRealCoordinates(self.bots[robotNumber - 1].robotPosition[0], self.bots[robotNumber - 1].robotPosition[1])
            x0 = np.array([conv_pos[0], conv_pos[1], self.bots[robotNumber - 1].globalOrientation]).reshape(-1, 1)

            self.bots[robotNumber - 1].mpc.x0 = x0
            u0 = self.bots[robotNumber - 1].mpc.make_step(x0)

            mpl.rcParams['font.size'] = 18
            mpl.rcParams['lines.linewidth'] = 3
            mpl.rcParams['axes.grid'] = True

            mpc_graphics = do_mpc.graphics.Graphics(self.bots[robotNumber - 1].mpc.data)
            
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

            conv_pos = self.ConvertOnRealCoordinates(self.bots[robotNumber - 1].robotPosition[0],
                                                     self.bots[robotNumber - 1].robotPosition[1])

            dtime = 1
            est_theta = self.bots[robotNumber - 1].globalOrientation + dtime * u0[1]

            x = conv_pos[0] + dtime * u0[0] * cos(est_theta)
            y = conv_pos[1] + dtime * u0[0] * sin(est_theta)

            neighbors = self.GetNeighbors(self.bots[robotNumber - 1].robotPosition)
            neighbors.append(self.bots[robotNumber-1].robotPosition)
            dist = math.inf
            index = 0
            min_index = 0

            pre_x = self.bots[robotNumber - 1].mpc.data.prediction(('_x', 'pos_x'))
            pre_y = self.bots[robotNumber - 1].mpc.data.prediction(('_x', 'pos_y'))

            pre_pos = []
            for i in range(len(pre_x[0])):
                pre_pos.append((pre_x[0][i][0], pre_y[0][i][0]))
                print('Prediction {}: {}, f: {}'.format(i, pre_pos[i], self.function(pre_pos[i][0], pre_pos[i][1])))

            for i in neighbors:
                conv_i = self.ConvertOnRealCoordinates(i[0], i[1])
                print('Coor: {}, dist: {}'.format(i, self.TaxicabNorm(conv_i, (x, y))))
                if self.TaxicabNorm(conv_i, (x, y)) < dist:
                    dist = self.TaxicabNorm(conv_i, (x, y))
                    self.bots[robotNumber - 1].robotPosition = i
                    min_index = index

                index += 1

            print('\nmin_index first: ', min_index)
            if min_index == len(neighbors)-1:
                index = 0
                min_index = 0
                dist = math.inf

                for i in neighbors:
                    conv_i = self.ConvertOnRealCoordinates(i[0], i[1])
                    temp_norm = self.TaxicabNorm(conv_i, (pre_pos[2][0], pre_pos[2][1]))
                    print('Coor for first prediction: {}, dist: {}'.format(i, temp_norm))
                    if temp_norm < dist:
                        dist = temp_norm
                        self.bots[robotNumber - 1].robotPosition = i
                        min_index = index
                        print('min_index first in for: ', min_index)

                    index += 1

            print('\nmin_index second: ', min_index)
            if min_index == len(neighbors) - 1:
                dist = math.inf

                for i in neighbors:
                    conv_i = self.ConvertOnRealCoordinates(i[0], i[1])
                    temp_norm = self.TaxicabNorm(conv_i, (pre_pos[3][0], pre_pos[3][1]))
                    print('Coor for second prediction: {}, dist: {}'.format(i, temp_norm))
                    if temp_norm < dist:
                        dist = temp_norm
                        self.bots[robotNumber - 1].robotPosition = i


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


            print('Controll: {}'.format(u0))

            print('x: ', x)
            print('y: ', y)
            print('Neighbors: ', neighbors)
            print('Robot{} pos: {}'.format(robotNumber, self.bots[robotNumber - 1].robotPosition))


            conv_pos = self.ConvertOnRealCoordinates(self.bots[robotNumber - 1].robotPosition[0],
                                                    self.bots[robotNumber - 1].robotPosition[1])
            self.bots[robotNumber - 1].sendCommand(robotNumber, str(RobotCommand.MOVE_WITH_ORIENTATION),
                                                   [conv_pos[0], conv_pos[1], est_theta[0]])

    def ConvertOnRealCoordinates(self, i, j):
        x_coord = [2.7, 2.4, 1.8, 1.2, 0.75, 0.3, -0.3, -0.75, -1.2, -1.8, -2.4, -2.7]
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



def main():
    rospy.init_node('testBot_controller')
    controller = Controller()
    while not rospy.is_shutdown():
        controller.mianLoop()
        rospy.sleep(0.01)


if __name__ == '__main__':
    main()
