#!/usr/bin/python3

import json
import sys
import enum
import math
from queue import PriorityQueue

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

from testBot_robot import ZoneColor
from testBot_robot import PuckColor
from testBot_robot import OrientationFromTag


def decode_command(sentence):
    command = json.loads(sentence)
    return command['name'], command['values']


def code_command(command_name, paramList):
    value = []
    for item in paramList:
        value.append(item)
    code = json.dumps({"name": command_name, "values": value})
    return code

class RobotCommand(enum.Enum):
    FORWARD = 'forward'
    TURN_LEFT = 'rotateLeft'
    TURN_RIGHT = 'rotateRight'
    ENTER_TO_ZONE = 'enterToZone'
    TAKE_PUCK = 'takeNearestPuck'
    ESCAPE_ZONE = 'escapeFromZone'
    PUT_DOWN_PUCK = 'putDownPuckOnPosition'
    GET_PUCK_COLOR = 'getPuckColor'
    GET_ORIENTATION = 'getOrientationFromTag'
    MOVE_TO_POSITION = 'moveToPosition'

    def __str__(self):
        return str(self.value)

class RobotState(enum.IntEnum):
    IDLE = 0
    FORWARD = 1
    TURN_LEFT = 2
    TURN_RIGHT = 3
    TAKE_PUCK = 4
    ESCAPE_ZONE = 5
    WAIT = 6
    PUT_DOWN_PUCK = 7
    READY = 8
    ENTER_ZONE = 9
    FINISH = 10

class RobotParameters:
    def __init__(self, robotName, position, orientation, callbackRobotAnswer):
        self.robotName = robotName
        self.robotState = RobotState.IDLE
        self.robotPosition = position
        self.puckColor = PuckColor.NONPUCK
        self.robotInZone = False
        self.zoneColor = ZoneColor.BLACK
        self.zoneNumber = 0
        self.globalOrientation = orientation
        self.path = []

        self.pubController = rospy.Publisher('/' + robotName + '/robotController', String, queue_size=10)
        self.pubRobotStop = rospy.Publisher("/" + robotName + "/robotStop", Bool, queue_size=10)
        rospy.Subscriber("/" + robotName + "/robotAnswer", String, callbackRobotAnswer)

    def sendCommand(self, commandName : String, paramList : list):
        self.pubController.publish(code_command(commandName, paramList))

class Controller:
    def __init__(self):
        self.puckInZone = dict()
        self.zoneEnterPosition = dict()
        self.bots = [RobotParameters('bot1', (4, 6), OrientationFromTag.EAST, self.callbackRobotAnswerBot1)]
        self.bots.append(RobotParameters('bot2', (5, 4), OrientationFromTag.SOUTH, self.callbackRobotAnswerBot2))
        self.botsFinished = [False, False]
        self.SetEnterZonePosition()
        self.SetPuckInZone()
        self.GenerateStartMap()


        print("Start")
        rospy.sleep(1)

    def mianLoop(self):

        for i in range(len(self.bots)):
            self.updateRobot(i+1)

        finishe = True
        for i in self.botsFinished:
            if not i:
                finishe = False
                break

        if finishe:
            print("Work finished !!!")
            exit(0)

    def updateRobot(self, robotNumber):
        robotState = self.bots[robotNumber-1].robotState

        if robotState == RobotState.FINISH:
            if self.botsFinished[robotNumber-1] == False:
                self.botsFinished[robotNumber-1] = True

        elif robotState == RobotState.IDLE:
             if self.bots[robotNumber-1].puckColor == PuckColor.NONPUCK:
                self.bots[robotNumber - 1].path = self.SearchPathToNearestZone(ZoneColor.BLACK, robotNumber, self.GenerateCostMapForRobot(robotNumber))
                self.bots[robotNumber - 1].zoneColor = ZoneColor.BLACK

                if len(self.bots[robotNumber - 1].path) == 1:
                    self.bots[robotNumber - 1].robotState = RobotState.ENTER_ZONE
                    #blokada strefy
                    self.bots[robotNumber-1].sendCommand(str(RobotCommand.ENTER_TO_ZONE), [self.bots[robotNumber-1].zoneColor])

                else:
                    self.bots[robotNumber - 1].robotState = RobotState.READY

        elif robotState == RobotState.ENTER_ZONE and self.bots[robotNumber-1].robotInZone:
            if self.bots[robotNumber-1].puckColor == PuckColor.NONPUCK:
                self.bots[robotNumber - 1].robotState = RobotState.TAKE_PUCK
                self.bots[robotNumber - 1].sendCommand(str(RobotCommand.TAKE_PUCK), [])

            else:
                self.bots[robotNumber - 1].robotState = RobotState.PUT_DOWN_PUCK
                x, y = self.calcPuckPositionInZone(self.bots[robotNumber - 1].zoneColor, self.bots[robotNumber - 1].zoneNumber)
                self.bots[robotNumber-1].sendCommand(str(RobotCommand.PUT_DOWN_PUCK), [x, y])

        elif robotState == RobotState.READY:
            path = self.bots[robotNumber - 1].path
            if len(path) > 1:
                self.MakeOneStepInPath2(robotNumber)

            elif len(path) == 1:
                self.bots[robotNumber - 1].robotState = RobotState.ENTER_ZONE
                self.bots[robotNumber - 1].sendCommand(str(RobotCommand.ENTER_TO_ZONE),
                                                       [self.bots[robotNumber - 1].zoneColor])

            elif len(path) == 0:
                self.bots[robotNumber - 1].robotState = RobotState.FINISH

        elif robotState == RobotState.WAIT:
            self.MakeOneStepInPath2(robotNumber)


    def callbackRobotAnswerBot1(self, msg):
        robotNumber = 1
        answer, answerParam = decode_command(msg.data)
        print("Robot: {}, answer {}, parma: {}".format(self.bots[robotNumber-1].robotName, answer, answerParam))

        if answer == str(RobotCommand.FORWARD):
            if len(answerParam) == 1:
                if answerParam[0] == True:
                    lastPos = self.bots[robotNumber-1].robotPosition
                    currentPos = self.bots[robotNumber-1].path[1]

                    self.robotMap[lastPos[0]][lastPos[1]] = 0
                    self.bots[robotNumber-1].path.remove(lastPos)
                    self.bots[robotNumber-1].robotPosition = currentPos

                    self.bots[robotNumber-1].robotState = RobotState.READY
                else:
                    assert answerParam == True, 'lost robot'

        elif answer == str(RobotCommand.MOVE_TO_POSITION):
            if len(answerParam) == 1:
                self.bots[robotNumber-1].globalOrientation = answerParam[0]
                lastPos = self.bots[robotNumber-1].robotPosition
                currentPos = self.bots[robotNumber-1].path[1]

                self.robotMap[lastPos[0]][lastPos[1]] = 0
                self.bots[robotNumber-1].path.remove(lastPos)
                self.bots[robotNumber-1].robotPosition = currentPos

                self.bots[robotNumber-1].robotState = RobotState.READY



        elif answer == str(RobotCommand.TURN_LEFT):
            if len(answerParam) == 1:
                if answerParam[0] != OrientationFromTag.NOT_SPECIFIED:
                    self.bots[robotNumber-1].globalOrientation = answerParam[0]
                    self.bots[robotNumber-1].robotState = RobotState.READY

                else:
                    self.bots[robotNumber-1].robotState = RobotState.TURN_RIGHT
                    self.bots[robotNumber-1].sendCommand(str(RobotCommand.TURN_RIGHT), [])

        elif answer == str(RobotCommand.TURN_RIGHT):
            if len(answerParam) == 1:
                if answerParam[0] != OrientationFromTag.NOT_SPECIFIED:
                    self.bots[robotNumber-1].globalOrientation = answerParam[0]
                    self.bots[robotNumber-1].robotState = RobotState.READY

                else:
                    self.bots[robotNumber-1].robotState = RobotState.TURN_LEFT
                    self.bots[robotNumber-1].sendCommand(str(RobotCommand.TURN_LEFT), [])

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
                self.bots[robotNumber-1].sendCommand(str(RobotCommand.ESCAPE_ZONE), [])

        elif answer == str(RobotCommand.ESCAPE_ZONE):
            if len(answerParam) == 1:
                self.bots[robotNumber-1].globalOrientation = answerParam[0]
                self.bots[robotNumber-1].robotInZone = False
                if self.bots[robotNumber-1].puckColor == PuckColor.NONPUCK:
                    self.bots[robotNumber-1].path = self.SearchPathToNearestZone(ZoneColor.BLACK, robotNumber, self.GenerateCostMapForRobot(robotNumber))
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

                self.bots[robotNumber-1].robotState = RobotState.READY


        elif answer == str(RobotCommand.PUT_DOWN_PUCK):
            if len(answerParam) == 1:
                if answerParam[0]:
                    self.puckInZone[self.bots[robotNumber-1].zoneColor][self.bots[robotNumber-1].zoneNumber] += 1
                    self.bots[robotNumber-1].puckColor = PuckColor.NONPUCK

                    self.bots[robotNumber-1].robotState = RobotState.ESCAPE_ZONE
                    self.bots[robotNumber-1].sendCommand(str(RobotCommand.ESCAPE_ZONE), [])

        elif answer == str(RobotCommand.GET_ORIENTATION):
            if len(answerParam) == 1:
                self.bots[robotNumber-1].globalOrientation = answerParam[0]

        elif answer == str(RobotCommand.GET_PUCK_COLOR):
            if len(answerParam) == 1:
                self.bots[robotNumber-1].puckColor = answerParam[0]

    def callbackRobotAnswerBot2(self, msg):
        robotNumber = 2
        answer, answerParam = decode_command(msg.data)
        print("Robot: {}, answer {}, parma: {}".format(self.bots[robotNumber - 1].robotName, answer, answerParam))

        if answer == str(RobotCommand.FORWARD):
            if len(answerParam) == 1:
                if answerParam[0] == True:
                    lastPos = self.bots[robotNumber - 1].robotPosition
                    currentPos = self.bots[robotNumber - 1].path[1]

                    self.robotMap[lastPos[0]][lastPos[1]] = 0
                    self.bots[robotNumber - 1].path.remove(lastPos)
                    self.bots[robotNumber - 1].robotPosition = currentPos

                    self.bots[robotNumber - 1].robotState = RobotState.READY
                else:
                    assert answerParam == True, 'lost robot'

        elif answer == str(RobotCommand.MOVE_TO_POSITION):
            if len(answerParam) == 1:
                self.bots[robotNumber-1].globalOrientation = answerParam[0]
                lastPos = self.bots[robotNumber-1].robotPosition
                currentPos = self.bots[robotNumber-1].path[1]

                self.robotMap[lastPos[0]][lastPos[1]] = 0
                self.bots[robotNumber-1].path.remove(lastPos)
                self.bots[robotNumber-1].robotPosition = currentPos

                self.bots[robotNumber-1].robotState = RobotState.READY


        elif answer == str(RobotCommand.TURN_LEFT):
            if len(answerParam) == 1:
                if answerParam[0] != OrientationFromTag.NOT_SPECIFIED:
                    self.bots[robotNumber - 1].globalOrientation = answerParam[0]
                    self.bots[robotNumber - 1].robotState = RobotState.READY

                else:
                    self.bots[robotNumber - 1].robotState = RobotState.TURN_RIGHT
                    self.bots[robotNumber - 1].sendCommand(str(RobotCommand.TURN_RIGHT), [])

        elif answer == str(RobotCommand.TURN_RIGHT):
            if len(answerParam) == 1:
                if answerParam[0] != OrientationFromTag.NOT_SPECIFIED:
                    self.bots[robotNumber - 1].globalOrientation = answerParam[0]
                    self.bots[robotNumber - 1].robotState = RobotState.READY

                else:
                    self.bots[robotNumber - 1].robotState = RobotState.TURN_LEFT
                    self.bots[robotNumber - 1].sendCommand(str(RobotCommand.TURN_LEFT), [])

        elif answer == str(RobotCommand.ENTER_TO_ZONE):
            if len(answerParam) == 1:
                if answerParam[0] != -1:
                    self.bots[robotNumber - 1].robotInZone = True
                    self.bots[robotNumber - 1].zoneNumber = answerParam[0]

        elif answer == str(RobotCommand.TAKE_PUCK):
            if len(answerParam) == 1:
                if answerParam[0] == PuckColor.NONPUCK:
                    self.puckInZone[self.bots[robotNumber - 1].zoneColor][self.bots[robotNumber - 1].zoneNumber] = 0

                self.bots[robotNumber - 1].puckColor = answerParam[0]

                self.bots[robotNumber - 1].robotState = RobotState.ESCAPE_ZONE
                self.bots[robotNumber - 1].sendCommand(str(RobotCommand.ESCAPE_ZONE), [])

        elif answer == str(RobotCommand.ESCAPE_ZONE):
            if len(answerParam) == 1:
                self.bots[robotNumber - 1].globalOrientation = answerParam[0]
                self.bots[robotNumber - 1].robotInZone = False
                if self.bots[robotNumber - 1].puckColor == PuckColor.NONPUCK:
                    self.bots[robotNumber - 1].path = self.SearchPathToNearestZone(ZoneColor.BLACK, robotNumber,
                                                                                   self.GenerateCostMapForRobot(robotNumber))
                    self.bots[robotNumber - 1].zoneColor = ZoneColor.BLACK

                else:
                    puckColor = self.bots[robotNumber - 1].puckColor
                    if puckColor == PuckColor.RED:
                        self.bots[robotNumber - 1].zoneColor = ZoneColor.RED

                    elif puckColor == PuckColor.GREEN:
                        self.bots[robotNumber - 1].zoneColor = ZoneColor.GREEN

                    elif puckColor == PuckColor.YELLOW:
                        self.bots[robotNumber - 1].zoneColor = ZoneColor.YELLOW

                    elif puckColor == PuckColor.BLUE:
                        self.bots[robotNumber - 1].zoneColor = ZoneColor.BLUE

                    self.bots[robotNumber - 1].path = self.SearchPathToNearestZone(self.bots[robotNumber - 1].zoneColor,
                                                                                   robotNumber,
                                                                                   self.GenerateCostMapForRobot(robotNumber))

                self.bots[robotNumber - 1].robotState = RobotState.READY


        elif answer == str(RobotCommand.PUT_DOWN_PUCK):
            if len(answerParam) == 1:
                if answerParam[0]:
                    self.puckInZone[self.bots[robotNumber - 1].zoneColor][self.bots[robotNumber - 1].zoneNumber] += 1
                    self.bots[robotNumber - 1].puckColor = PuckColor.NONPUCK

                    self.bots[robotNumber - 1].robotState = RobotState.ESCAPE_ZONE
                    self.bots[robotNumber - 1].sendCommand(str(RobotCommand.ESCAPE_ZONE), [])

        elif answer == str(RobotCommand.GET_ORIENTATION):
            if len(answerParam) == 1:
                self.bots[robotNumber - 1].globalOrientation = answerParam[0]

        elif answer == str(RobotCommand.GET_PUCK_COLOR):
            if len(answerParam) == 1:
                self.bots[robotNumber - 1].puckColor = answerParam[0]

    def calcPuckPositionInZone(self, zoneColor: ZoneColor, zoneNumber):
        puckNumberInZone = self.puckInZone[zoneColor][zoneNumber]
        xPos = 0
        yPos = 0

        if zoneNumber == 2:
            if puckNumberInZone <= 24:
                x = puckNumberInZone % 4
                y = math.floor(puckNumberInZone/4.0)
                xPos = 0.1 + x*0.15
                yPos = 0.1 + y*0.15

        elif zoneNumber == 1:
            if puckNumberInZone <= 24:
                if puckNumberInZone <= 7:
                    x = puckNumberInZone % 4
                    y = math.floor(puckNumberInZone / 4.0)
                    xPos = 0.1 + x * 0.15
                    yPos = 0.1 + y * 0.15

                elif puckNumberInZone >= 8 and puckNumberInZone <= 15:
                    x = puckNumberInZone % 4
                    y = math.floor(puckNumberInZone / 4.0) - 2
                    xPos = 0.1 + x * 0.15
                    yPos = 1.15 - y * 0.15

                else:
                    x = puckNumberInZone % 2
                    y = math.floor(puckNumberInZone / 2.0) - 8
                    xPos = 0.1 + x * 0.15
                    yPos = 0.4 + y * 0.15

        return -yPos, -xPos



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
            j = j + 1

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
                    costMap[i].append(100)

                elif self.robotMap[i][j] == robotNumber:
                    costMap[i].append(0)

                else:
                    costMap[i].append(1)

        #Dodac koszty przeciecia sie tras

        return costMap

    def SearchPathToNearestZone(self, zoneColor: ZoneColor, robotNumber, costMap):
        path = []
        cost = math.inf
        if zoneColor == ZoneColor.BLACK:
            for zoneNumber in range(1,5):
                if self.puckInZone[zoneColor][zoneNumber] == 1:
                    tempPath, tempCost = self.SearchPathToZone(zoneColor, zoneNumber, robotNumber, costMap)
                    print("Path to black zone number: {}, cost: {}".format(zoneNumber, tempCost))
                    if tempCost < cost:
                        path = tempPath
                        cost = tempCost

        else:
            for zoneNumber in range(1, 3):
                if self.puckInZone[zoneColor][zoneNumber] <= 24:
                    tempPath, tempCost = self.SearchPathToZone(zoneColor, zoneNumber, robotNumber, costMap)
                    print("Path to color zone number: {}, cost: {}".format(zoneNumber, tempCost))
                    if tempCost < cost:
                        path = tempPath
                        cost = tempCost

        print("Path: ", path)
        return path


    def SearchPathToZone(self, zoneColor: ZoneColor, zoneNumber, robotNumber, costMap):
        goalPos = self.zoneEnterPosition[zoneColor][zoneNumber]

        frontier = PriorityQueue()
        frontier.put(self.bots[robotNumber-1].robotPosition, 0)
        cameFrom = dict()
        currentCost = dict()

        cameFrom[self.bots[robotNumber-1].robotPosition] = None
        currentCost[self.bots[robotNumber-1].robotPosition] = 0

        while not frontier.empty():
            currentPos = frontier.get()

            if currentPos == goalPos:
                break

            for next in self.GetNeighbors(currentPos):
                new_cost = currentCost[currentPos] + costMap[next[0]][next[1]]
                if next not in currentCost or new_cost < currentCost[next]:
                    currentCost[next] = new_cost
                    priority = new_cost + self.TaxicabNorm(next, goalPos)
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


    def TaxicabNorm(self, position1, position2):
        return abs(position2[0] - position1[0]) + abs(position2[1] - position1[1])

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

    def MakeOneStepInPath2(self, robotNumber):
        robotState = self.bots[robotNumber - 1].robotState
        if len(self.bots[robotNumber - 1].path) >= 2 and (robotState == RobotState.READY or robotState == RobotState.WAIT):

            nextPos = self.bots[robotNumber - 1].path[1]

            if self.robotMap[nextPos[0]][nextPos[1]] == 0:
                self.robotMap[nextPos[0]][nextPos[1]] = robotNumber
                self.bots[robotNumber - 1].robotState = RobotState.FORWARD
                x,y = self.ConvertOnRealCoordinates(nextPos[0], nextPos[1])
                self.bots[robotNumber - 1].sendCommand(str(RobotCommand.MOVE_TO_POSITION), [x,y])

            else:
                self.bots[robotNumber - 1].robotState = RobotState.WAIT
                return


    def MakeOneStepInPath(self, robotNumber):
        robotState = self.bots[robotNumber - 1].robotState
        if len(self.bots[robotNumber-1].path) >= 2 and (robotState == RobotState.READY or robotState == RobotState.WAIT):
            currentPos = self.bots[robotNumber-1].robotPosition
            nextPos = self.bots[robotNumber-1].path[1]
            currentOrient = self.bots[robotNumber-1].globalOrientation

            needOrientation = currentOrient

            if currentPos[0] > nextPos[0]:
                needOrientation = OrientationFromTag.WEST

            elif currentPos[0] < nextPos[0]:
                needOrientation = OrientationFromTag.EAST

            elif currentPos[1] > nextPos[1]:
                needOrientation = OrientationFromTag.NORTH

            elif currentPos[1] < nextPos[1]:
                needOrientation = OrientationFromTag.SOUTH

            if needOrientation != currentOrient:
                if needOrientation == OrientationFromTag.WEST and (currentOrient == OrientationFromTag.NORTH or
                currentOrient == OrientationFromTag.EAST):
                    self.bots[robotNumber - 1].robotState = RobotState.TURN_LEFT
                    self.bots[robotNumber - 1].sendCommand(str(RobotCommand.TURN_LEFT), [])

                elif needOrientation == OrientationFromTag.WEST and currentOrient == OrientationFromTag.SOUTH:
                    self.bots[robotNumber - 1].robotState = RobotState.TURN_RIGHT
                    self.bots[robotNumber - 1].sendCommand(str(RobotCommand.TURN_RIGHT), [])

                elif needOrientation == OrientationFromTag.NORTH and (currentOrient == OrientationFromTag.EAST or
                    currentOrient == OrientationFromTag.SOUTH):
                    self.bots[robotNumber - 1].robotState = RobotState.TURN_LEFT
                    self.bots[robotNumber - 1].sendCommand(str(RobotCommand.TURN_LEFT), [])

                elif needOrientation == OrientationFromTag.NORTH and currentOrient == OrientationFromTag.WEST:
                    self.bots[robotNumber - 1].robotState = RobotState.TURN_RIGHT
                    self.bots[robotNumber - 1].sendCommand(str(RobotCommand.TURN_RIGHT), [])

                elif needOrientation == OrientationFromTag.EAST and (currentOrient == OrientationFromTag.SOUTH or
                    currentOrient == OrientationFromTag.WEST):
                    self.bots[robotNumber - 1].robotState = RobotState.TURN_LEFT
                    self.bots[robotNumber - 1].sendCommand(str(RobotCommand.TURN_LEFT), [])

                elif needOrientation == OrientationFromTag.EAST and currentOrient == OrientationFromTag.NORTH:
                    self.bots[robotNumber - 1].robotState = RobotState.TURN_RIGHT
                    self.bots[robotNumber - 1].sendCommand(str(RobotCommand.TURN_RIGHT), [])

                elif needOrientation == OrientationFromTag.SOUTH and (currentOrient == OrientationFromTag.WEST or
                    currentOrient == OrientationFromTag.NORTH):
                    self.bots[robotNumber - 1].robotState = RobotState.TURN_LEFT
                    self.bots[robotNumber - 1].sendCommand(str(RobotCommand.TURN_LEFT), [])

                elif needOrientation == OrientationFromTag.SOUTH and currentOrient == OrientationFromTag.EAST:
                    self.bots[robotNumber - 1].robotState = RobotState.TURN_RIGHT
                    self.bots[robotNumber - 1].sendCommand(str(RobotCommand.TURN_RIGHT), [])

            else:
                if self.robotMap[nextPos[0]][nextPos[1]] == 0:
                    self.robotMap[nextPos[0]][nextPos[1]] = robotNumber
                    self.bots[robotNumber - 1].robotState = RobotState.FORWARD
                    self.bots[robotNumber - 1].sendCommand(str(RobotCommand.FORWARD), [])

                else:
                    self.bots[robotNumber-1].robotState = RobotState.WAIT
                    return

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


def main():
    rospy.init_node('testBot_controller')
    controller = Controller()
    while not rospy.is_shutdown():
        controller.mianLoop()


if __name__ == '__main__':
    main()
